/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ld_dvfs_handler.h"
#include "ld_dvfs.h"
#include "hal/nrf_hsfll.h"
#include <nrfs_dvfs.h>
#include <nrfs_backend_ipc_service.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(LD_DVFS_LIB, CONFIG_LOCAL_DOMAIN_DVFS_LIB_LOG_LEVEL);

K_SEM_DEFINE(dvfs_service_sync_sem, 0, 1);
K_SEM_DEFINE(dvfs_service_idle_sem, 0, 1);

#define DVFS_SERV_HDL_INIT_DONE_BIT_POS (0)
#define DVFS_SERV_HDL_FREQ_CHANGE_IN_PROGRESS_BIT_POS (1)

atomic_t dvfs_service_handler_state_bits;

static enum dvfs_frequency_setting current_freq_setting;

/**
 * @brief Helper function to set bit in state variable.
 *
 * @param bit_pos bit position in state variable.
 */
static void dvfs_service_handler_set_state_bit(uint32_t bit_pos)
{
	atomic_set_bit(&dvfs_service_handler_state_bits, bit_pos);
}

/**
 * @brief Helper function to clear bit in state variable.
 *
 * @param bit_pos bit position in state variable.
 */
static void dvfs_service_handler_clear_state_bit(uint32_t bit_pos)
{
	atomic_clear_bit(&dvfs_service_handler_state_bits, bit_pos);
}

/**
 * @brief Helper function to get dvfs handler state bit
 *
 * @param bit_pos bit position in state variable
 * @return true bit is set
 * @return false bit is not set
 */
static bool dvfs_service_handler_get_state_bit(uint32_t bit_pos)
{
	return atomic_test_bit(&dvfs_service_handler_state_bits, bit_pos);
}

/**
 * @brief Function to check if dvfs init has been done.
 *
 * @return true init done
 * @return false init not yet completed
 */
static bool dvfs_service_handler_init_done(void)
{
	return dvfs_service_handler_get_state_bit(DVFS_SERV_HDL_INIT_DONE_BIT_POS);
}

/**
 * @brief Function to check if oppoint change is in progress.
 *
 * @return true oppoint change in progress.
 * @return false no oppoint change was requested.
 */
static bool dvfs_service_handler_freq_change_in_progress(void)
{
	return dvfs_service_handler_get_state_bit(DVFS_SERV_HDL_FREQ_CHANGE_IN_PROGRESS_BIT_POS);
}

/**
 * @brief Function called if nrfs error handling is needed.
 *
 * @param err error number.
 */
static void dvfs_service_handler_nrfs_error_check(nrfs_err_t err)
{
	if (err != NRFS_SUCCESS) {
		LOG_ERR("Failed with nrfs error: %d", err);
	}
}

/**
 * @brief Function called if dvfs general error handling is needed.
 *
 * @param err error.
 */
static void dvfs_service_handler_error(int err)
{
	if (err != 0) {
		LOG_ERR("Failed with error: %d", err);
	}
}

/**
 * @brief Get the next context object helper function
 *
 * @return uint32_t* context for nrfs message.
 */
static uint32_t *get_next_context(void)
{
	static uint32_t ctx;

	ctx++;
	return &ctx;
}

/**
 * @brief Check if frequency setting is allowed.
 *
 * @return true this frequency setting is supported.
 * @return false not supported frequency setting.
 */
static bool dvfs_service_handler_freq_setting_allowed(enum dvfs_frequency_setting freq_setting)
{
	if (freq_setting == DVFS_FREQ_HIGH || freq_setting == DVFS_FREQ_MEDLOW ||
	    freq_setting == DVFS_FREQ_LOW) {
		return true;
	}

	return false;
}

/**
 * @brief Get current oppoint from current frequency.
 *
 * @return enum dvfs_oppoint current oppoint
 */
static enum dvfs_frequency_setting dvfs_service_handler_get_current_oppoint(void)
{
	LOG_INF("Current LD freq setting: %d", current_freq_setting);
	return current_freq_setting;
}

/**
 * @brief Function to check if current operation is down-scaling
 *
 * @param target_freq_setting frequency setting target
 * @return true down-scaling procedure
 * @return false not down-scaling procedure
 */
static bool dvfs_service_handler_is_downscaling(enum dvfs_frequency_setting target_freq_setting)
{
	if (dvfs_service_handler_freq_setting_allowed(target_freq_setting)) {
		LOG_DBG("Checking if downscaling %s",
			(dvfs_service_handler_get_current_oppoint() < target_freq_setting) ? "YES" :
											     "NO");
		return dvfs_service_handler_get_current_oppoint() < target_freq_setting;
	}

	return false;
}

/**
 * @brief Function handling steps for scaling preparation.
 *
 * @param oppoint_freq requested frequency oppoint
 */
static void dvfs_service_handler_prepare_to_scale(enum dvfs_frequency_setting oppoint_freq)
{
	LOG_INF("Prepare to scale, oppoint freq %d", oppoint_freq);
	enum dvfs_frequency_setting new_oppoint	    = oppoint_freq;
	enum dvfs_frequency_setting current_oppoint = dvfs_service_handler_get_current_oppoint();

	if (new_oppoint == current_oppoint) {
		LOG_INF("New oppoint is same as previous, no change");
	} else {
		ld_dvfs_configure_abb_for_transition(current_oppoint, new_oppoint);

		if (dvfs_service_handler_is_downscaling(new_oppoint)) {
			int32_t err = ld_dvfs_configure_hsfll(new_oppoint);

			if (err != 0) {
				dvfs_service_handler_error(err);
			}
		}
	}
}

/**
 * @brief Do background job during scaling process
 *	  (for example increased power consumption during down-scale)
 * @param oppoint_freq requested frequency oppoint
 */
static void dvfs_service_handler_scaling_background_job(enum dvfs_frequency_setting oppoint_freq)
{
	LOG_INF("Perform scaling background job if needed.");
	if (dvfs_service_handler_is_downscaling(oppoint_freq)) {
		k_sem_give(&dvfs_service_idle_sem);
	}
}

/**
 * @brief Perform scaling finnish procedure.
 *
 * @param oppoint_freq target frequency oppoint.
 */
static void dvfs_service_handler_scaling_finish(enum dvfs_frequency_setting oppoint_freq)
{
	LOG_INF("Scaling finnish oppoint freq %d", oppoint_freq);
	ld_dvfs_scaling_finish(dvfs_service_handler_is_downscaling(oppoint_freq));
	if (!dvfs_service_handler_is_downscaling(oppoint_freq)) {
		int32_t err = ld_dvfs_configure_hsfll(oppoint_freq);

		if (err != 0) {
			dvfs_service_handler_error(err);
		}
	}
	current_freq_setting = oppoint_freq;
}

/**
 * @brief Function to set hsfll to highest frequency when switched to ABB.
 *
 */
static void dvfs_service_handler_set_initial_hsfll_config(void)
{
	int32_t err = ld_dvfs_wait_abb_statusana_locked();

	if (err != 0) {
		dvfs_service_handler_error(err);
	} else {
		err		     = ld_dvfs_configure_hsfll(DVFS_FREQ_HIGH);
		current_freq_setting = DVFS_FREQ_HIGH;

		if (err != 0) {
			dvfs_service_handler_error(err);
		}
	}
}

/**
 * @brief DVFS event handler callback function.
 *
 * @param p_evt event to handle
 * @param context context data
 */
void nrfs_dvfs_evt_handler(nrfs_dvfs_evt_t const *p_evt, void *context)
{
	LOG_INF("%s", __func__);
	switch (p_evt->type) {
	case NRFS_DVFS_EVT_INIT_PREPARATION:
		LOG_INF("DVFS handler EVT_INIT_PREPARATION");
#if defined(NRF_SECURE)
		ld_dvfs_clear_zbb();
		dvfs_service_handler_nrfs_error_check(
			nrfs_dvfs_init_complete_request(get_next_context()));
		LOG_INF("DVFS handler EVT_INIT_PREPARATION handled");
#else
		LOG_ERR("DVFS handler - unexpected EVT_INIT_PREPARATION");
#endif
		break;
	case NRFS_DVFS_EVT_INIT_DONE:
		LOG_INF("DVFS handler EVT_INIT_DONE");
		dvfs_service_handler_set_initial_hsfll_config();
		dvfs_service_handler_set_state_bit(DVFS_SERV_HDL_INIT_DONE_BIT_POS);
		k_sem_give(&dvfs_service_sync_sem);
		LOG_INF("DVFS handler EVT_INIT_DONE handled");
		break;
	case NRFS_DVFS_EVT_OPPOINT_REQ_CONFIRMED:
		/* Optional confirmation from sysctrl, wait for oppoint.*/
		LOG_INF("DVFS handler EVT_OPPOINT_REQ_CONFIRMED");
		break;
	case NRFS_DVFS_EVT_OPPOINT_SCALING_PREPARE:
		/*Target oppoint will be received here.*/
		LOG_INF("DVFS handler EVT_OPPOINT_SCALING_PREPARE");
#if !defined(NRF_SECURE)
		if (dvfs_service_handler_is_downscaling(p_evt->freq)) {
#endif
			dvfs_service_handler_prepare_to_scale(p_evt->freq);
			dvfs_service_handler_nrfs_error_check(
						nrfs_dvfs_ready_to_scale(get_next_context()));
			dvfs_service_handler_scaling_background_job(p_evt->freq);
			LOG_INF("DVFS handler EVT_OPPOINT_SCALING_PREPARE handled");
#if !defined(NRF_SECURE)
			current_freq_setting = p_evt->freq;
		} else {
			LOG_ERR("DVFS handler - unexpected EVT_OPPOINT_SCALING_PREPARE");
		}
#endif
		break;
	case NRFS_DVFS_EVT_OPPOINT_SCALING_DONE:
		LOG_INF("DVFS handler EVT_OPPOINT_SCALING_DONE");
		dvfs_service_handler_clear_state_bit(DVFS_SERV_HDL_FREQ_CHANGE_IN_PROGRESS_BIT_POS);
		dvfs_service_handler_scaling_finish(p_evt->freq);
		LOG_INF("DVFS handler EVT_OPPOINT_SCALING_DONE handled");
		break;
	case NRFS_DVFS_EVT_REJECT:
		LOG_ERR("DVFS handler - request rejected");
		break;
	default:
		LOG_ERR("DVFS handler - unexpected event: 0x%x", p_evt->type);
		break;
	}
}

/**
 * @brief Task to handle dvfs init procedure.
 *
 * @param dummy0
 * @param dummy1
 * @param dummy2
 */
static void dvfs_service_handler_task(void *dummy0, void *dummy1, void *dummy2)
{
	ARG_UNUSED(dummy0);
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);

	LOG_INF("Trim ABB for default voltage.");
	ld_dvfs_init();

	LOG_INF("Waiting for backend init");
	/* Wait for ipc initialization */
	nrfs_backend_wait_for_connection(K_FOREVER);

	nrfs_err_t status;

	LOG_INF("nrfs_dvfs_init");
	status = nrfs_dvfs_init(nrfs_dvfs_evt_handler);
	dvfs_service_handler_nrfs_error_check(status);

	LOG_INF("nrfs_dvfs_init_prepare_request");
	status = nrfs_dvfs_init_prepare_request(get_next_context());
	dvfs_service_handler_nrfs_error_check(status);

	/* Wait for init*/
	k_sem_take(&dvfs_service_sync_sem, K_FOREVER);

	LOG_INF("DVFS init done.");

#if defined(CONFIG_LOCAL_DOMAIN_DVFS_SCALE_DOWN_AFTER_INIT)
	LOG_INF("Requesting lowest frequency oppoint.");
	dvfs_service_handler_change_freq_setting(DVFS_FREQ_LOW);
#endif

	while (1) {
		k_sem_take(&dvfs_service_idle_sem, K_FOREVER);
		/* perform background processing */
		ld_dvfs_scaling_background_process(true);
	}
}
K_THREAD_DEFINE(dvfs_service_handler_task_id,
		CONFIG_LOCAL_DOMAIN_DVFS_HANDLER_TASK_STACK_SIZE,
		dvfs_service_handler_task,
		NULL,
		NULL,
		NULL,
		CONFIG_LOCAL_DOMAIN_DVFS_HANDLER_TASK_PRIORITY,
		0,
		0);

int32_t dvfs_service_handler_change_freq_setting(enum dvfs_frequency_setting freq_setting)
{
	if (dvfs_service_handler_init_done()) {
		if (!dvfs_service_handler_freq_change_in_progress()) {
			if (dvfs_service_handler_freq_setting_allowed(freq_setting)) {
				nrfs_err_t status =
					nrfs_dvfs_oppoint_request(freq_setting, get_next_context());

				dvfs_service_handler_nrfs_error_check(status);
				return status;
			}
			return -ENXIO;
		}

		LOG_INF("Frequency change in progress.");
		return -EBUSY;
	}

	LOG_INF("Init not done!");
	return -EAGAIN;
}
