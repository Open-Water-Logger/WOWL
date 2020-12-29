/*
	wowlBle.c
	
	The Bluetooth Low Energy interface module for the WOWL.
	
	2020-09-22	WHF	 Created.
*/

// Somehow, not specifying NDEBUG for this module reduces the effective clock
//  rate of the processor (as measured by WAIT_USEC) by a factor of two.
//  TODO: determine how and fix.
#ifndef NDEBUG
#	undef  _DEBUG
#	define NDEBUG
#endif


#include "wowl.h"
#include "wowlCodes.h"

// BLE Common:
#include <ble_advdata.h>
#include <ble_advertising.h>
#include <ble_conn_params.h>
#include <ble_conn_state.h>
#include <ble_hci.h>

// Services:
#include <ble_dis/ble_dis.h>
#include <ble_lbs/ble_lbs.h>

// System:
#include <nrf.h>
#include <nrf_delay.h>
#include <softdevice_handler.h>

// Components:
#include <timer/app_timer.h>
#include <util/app_error.h>
#include <nrf_gpio.h>

#include <fds.h>
#include <fstorage.h>
#include <peer_manager.h>


//********************************	Constants  *******************************//

// Number of central links used by the application.
//	When changing this number remember to adjust the RAM settings.
#define CENTRAL_LINK_COUNT												  0

// Number of peripheral links used by the application.
//	When changing this number remember to adjust the RAM settings.
#define PERIPHERAL_LINK_COUNT											  1

// The advertising interval, in msec.  By the standard it can range
//   from 20 to 10240 msecs, in 0.625 msec steps.
// This was originally 187.5 msec.  Increasing to one second saves
//  about 0.2 mA, but appears to make the connecting process less reliable.
#define APP_ADV_INTERVAL_MSEC		                                  187.5
// The advertising interval in units of 0.625 ms.
#define APP_ADV_INTERVAL			((int) (0.625 * APP_ADV_INTERVAL_MSEC))

// Amount of time spent advertising.  0 => timeout disabled
//  Ranges from 1 to 0x3FFF.  See ble_gap_adv_params_t.
#define APP_ADV_TIMEOUT_IN_SECONDS	                                     0u


// Minimum acceptable connection interval.  Between 7.5 and 4000 msec.
#define MIN_CONN_INTERVAL_MSEC                                        100.0

// Maximum acceptable connection interval
#define MAX_CONN_INTERVAL_MSEC				                         1000.0


#define SLAVE_LATENCY													  0

// Connection supervisory time-out (4 seconds).
#define CONN_SUP_TIMEOUT					MSEC_TO_UNITS(4000, UNIT_10_MS)

// Time from initiating event (connect or start of notification) to first time
//	sd_ble_gap_conn_param_update is called.
#define FIRST_CONN_PARAMS_UPDATE_DELAY	\
		APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
		
// Time between each call to sd_ble_gap_conn_param_update after the first call.		
#define NEXT_CONN_PARAMS_UPDATE_DELAY  \
		APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
		
// Number of attempts before giving up the connection parameter negotiation.
#define MAX_CONN_PARAMS_UPDATE_COUNT									   3



/////  Device Info Service Data	 /////
#define MANUFACTURER_NAME				"Creare"
#define MODEL_NUM						"1010309-900-0011"
#define HARDWARE_REV					"Rev. A"
// Manufacturer ID, part of System ID. Will be passed to 
//	Device Information Service.
#define MANUFACTURER_ID					0x1122334455
// Organizational Unique ID, part of System ID. Will be passed to 
//	Device Information Service.
#define ORG_UNIQUE_ID					0x667788


#if (NRF_SD_BLE_API_VERSION == 3)
// MTU size used in the softdevice enabling and to reply to a 
//  BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event.
#define NRF_BLE_MAX_MTU_SIZE			GATT_MTU_SIZE_DEFAULT
#endif

// Reply when unsupported features are requested.
#define APP_FEATURE_NOT_SUPPORTED		BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2


/////  Security Settings  /////

// Perform bonding.
#define SEC_PARAM_BOND					                                   1
// Man In The Middle protection not required.
#define SEC_PARAM_MITM					                                   0
// LE Secure Connections not enabled.                                      
#define SEC_PARAM_LESC					                                   0
// Keypress notifications not enabled.                                     
#define SEC_PARAM_KEYPRESS				                                   0
// No I/O capabilities.                                                    
#define SEC_PARAM_IO_CAPABILITIES		                BLE_GAP_IO_CAPS_NONE
// Out Of Band data not available.                                         
#define SEC_PARAM_OOB					                                   0
// Minimum encryption key size.                                            
#define SEC_PARAM_MIN_KEY_SIZE			                                   7
// Maximum encryption key size.                                            
#define SEC_PARAM_MAX_KEY_SIZE			                                  16


//*******************************  Module Data	******************************//

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;						
// Handle of the current connection.

 // Universally unique service identifiers. */
static ble_uuid_t m_adv_uuids[] = {
	{ BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE },
	{ WOWL_SERVICE_UUID, BLE_UUID_TYPE_BLE },
};

static uint8_t writeBuffer[1024];

// Number of packets currently pending transmission: 
static uint8_t nTxPending;

//***********************  Local Function Declarations	**********************//

static void gap_params_init(void);
static void advertising_init(void);
static void services_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void conn_params_init(void);
static void advertising_start(void);
static void on_ble_evt(ble_evt_t * p_ble_evt);
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void ble_stack_init(void);

static void peer_manager_init(bool erase_bonds);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);

//****************************	Global Functions  ****************************//
void wowlBleIdle(void)
{
	// Wait for an event.  Before doing so, clear any pending 
	//  floating point flags, as they will erroneously wake the processor.
	// Clear FPU flags:
	const uint32_t fpuFlags = __get_FPSCR() & ~0x9F;
	__set_FPSCR(fpuFlags);
	// Ensure propagation:
	(void) __get_FPSCR();
	// Clear the FPU interrupt:
	NVIC_ClearPendingIRQ(FPU_IRQn);
	
nrf_gpio_pin_set(PIN_TP20);

	// Now idle the processor until an event occurs:
	asm(" WFI");

nrf_gpio_pin_clear(PIN_TP20);
}

// Initialize the Bluetooth stack:
void wowlBleInit(void)
{
	// Initialize sub-modules.
	
	ble_stack_init();

	if (!DO_TESTS) {
		peer_manager_init(false);
		gap_params_init();
		advertising_init();
		services_init();
		conn_params_init();
	
		// Begin advertising.
		advertising_start();
	} else {
		// Running Bluetooth is incompatible with the unit tests.
	}	
}

// Notify the host that a characteristic has changed.
bool wowlBleSendNotification(uint16_t valueHandle, void* pData, uint16_t len)
{
	bool ok = true;
	ble_gatts_hvx_params_t params;
	uint16_t lenInOut = 0u;
	uint32_t err;
	
	uint8_t backup[20], * const pD = (uint8_t*) pData;
	
	memcpy(backup, pData, sizeof(backup));
	
	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_NOTIFICATION;
	params.handle = valueHandle;
	params.p_len = &lenInOut;
	
	// The number of bytes transmitted is limited by the MTU.  Thus, we loop,
	//   transmitting 20 bytes each time.
	while (params.offset < len) {
		const uint16_t nTx = MIN(len - params.offset, sizeof(backup));
		if (params.offset > 0) {
			// Copy from attribute into first chunk of bytes:
			memcpy(pD, &pD[params.offset], nTx);
			// Zero any leftovers.
			memset(&pD[nTx], 0u, sizeof(backup) - nTx);
		} else {
			// Do not copy on first pass.
		}
		err = sd_ble_gatts_hvx(m_conn_handle, &params);
		params.offset += nTx;
		assert(err == NRF_SUCCESS);
		if (err != NRF_SUCCESS) {
			ok = false;
			break;
		} else {
			// The packet write was successful.  Increment the outstanding
			//  packet counter.
			++nTxPending;
			// Continue transmitting.
		}
	}
	// Restore the backup.
	memcpy(pData, backup, sizeof(backup));
	
	return ok;
}

bool wowlBleTxBusy(void) { return nTxPending > 0u; }

void wowlBleUpdateValueSize(uint16_t valueHandle, uint16_t len)
{
	ble_gatts_value_t valueSizeUpdate = {
		.len = len,
		.offset = 0U,
		.p_value = NULL
	};
	uint32_t err;
	
	err = sd_ble_gatts_value_set(m_conn_handle, valueHandle, &valueSizeUpdate);
	
	assert(err == NRF_SUCCESS);
}

//***********************  Local Function Definitions  ***********************//


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)
 *           parameters of the
 *			device including the device name, appearance, and the preferred
 *          connection parameters.
 */
static void gap_params_init(void)
{
	uint32_t				err_code;
	ble_gap_conn_params_t	gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;
	
	// Read the serial number from non-volatile memory
	//   (guaranteed non-NULL):
	const char *pId = "WOWL";

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
										  (const uint8_t *) pId,
										  strlen(pId));
	APP_ERROR_CHECK(err_code);

	// This product does not fit any of the predifined appearances.
	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MSEC_TO_UNITS(
			MIN_CONN_INTERVAL_MSEC,
			UNIT_1_25_MS
	);
	gap_conn_params.max_conn_interval = MSEC_TO_UNITS(
			MAX_CONN_INTERVAL_MSEC,
			UNIT_1_25_MS
	);
	gap_conn_params.slave_latency	  = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *			Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
	uint32_t			   err_code;
	ble_advdata_t		   advdata;
	ble_adv_modes_config_t options;

	// Build advertising data struct to pass into @ref ble_advertising_init.
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type				= BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance		= true;
	advdata.flags					= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	advdata.uuids_complete.p_uuids	= m_adv_uuids;

	memset(&options, 0, sizeof(options));
	options.ble_adv_fast_enabled  = true;
	options.ble_adv_fast_interval = APP_ADV_INTERVAL;
	options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	uint32_t	   err_code;
	ble_dis_init_t	 dis_init;
	ble_dis_sys_id_t sys_id;
	
	// Initialize Device Information Service.
	memset(&dis_init, 0, sizeof(dis_init));

	ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
	ble_srv_ascii_to_utf8(&dis_init.model_num_str,	   MODEL_NUM);
	ble_srv_ascii_to_utf8(&dis_init.hw_rev_str,		   HARDWARE_REV);
	ble_srv_ascii_to_utf8(&dis_init.fw_rev_str,		   __DATE__ " " __TIME__);
	dis_init.sw_rev_str = dis_init.fw_rev_str;
	
	sys_id.manufacturer_id			  = MANUFACTURER_ID;
	sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
	dis_init.p_sys_id				  = &sys_id;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

	err_code = ble_dis_init(&dis_init);
	APP_ERROR_CHECK(err_code);
	
	// Initialize the WOWL service:
	wowlServiceInit();
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection
 *   Parameters Module that are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by
 *    simply setting the disconnect_on_fail config parameter, 
 *    but instead we use the event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt	 Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
	uint32_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		err_code = sd_ble_gap_disconnect(
				m_conn_handle,
				BLE_HCI_CONN_INTERVAL_UNACCEPTABLE
		);
		APP_ERROR_CHECK(err_code);
	}
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt	 Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
	ret_code_t err_code;

	switch (p_evt->evt_id)
	{
		case PM_EVT_BONDED_PEER_CONNECTED:
		{
			err_code = pm_peer_rank_highest(p_evt->peer_id);
			if (err_code != NRF_ERROR_BUSY)
			{
				APP_ERROR_CHECK(err_code);
			}
		} break; // PM_EVT_BONDED_PEER_CONNECTED

		case PM_EVT_CONN_SEC_START:
			break; // PM_EVT_CONN_SEC_START

		case PM_EVT_CONN_SEC_SUCCEEDED:
		{
			err_code = pm_peer_rank_highest(p_evt->peer_id);
			if (err_code != NRF_ERROR_BUSY)
			{
				APP_ERROR_CHECK(err_code);
			}
		} break; // PM_EVT_CONN_SEC_SUCCEEDED

		case PM_EVT_CONN_SEC_FAILED:
		{
			/** In some cases, when securing fails, it can be restarted directly. Sometimes it can
			 *	be restarted, but only after changing some Security Parameters. Sometimes, it cannot
			 *	be restarted until the link is disconnected and reconnected. Sometimes it is
			 *	impossible, to secure the link, or the peer device does not support it. How to
			 *	handle this error is highly application dependent. */
			switch (p_evt->params.conn_sec_failed.error)
			{
				case PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING:
					// Rebond if one party has lost its keys.
					err_code = pm_conn_secure(p_evt->conn_handle, true);
					if (err_code != NRF_ERROR_INVALID_STATE)
					{
						APP_ERROR_CHECK(err_code);
					}
					break; // PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING

				default:
					break;
			}
		} break; // PM_EVT_CONN_SEC_FAILED

		case PM_EVT_CONN_SEC_CONFIG_REQ:
		{
			// Reject pairing request from an already bonded peer.
			pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
			pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
		} break; // PM_EVT_CONN_SEC_CONFIG_REQ

		case PM_EVT_STORAGE_FULL: {
			// There is no more room to store peer data.
			// Run garbage collection on the flash.
			err_code = fds_gc();
			if (err_code == FDS_ERR_BUSY 
					|| err_code == FDS_ERR_NO_SPACE_IN_QUEUES) {
				// Retry.
			} else {
				APP_ERROR_CHECK(err_code);
			} 
		} break; // PM_EVT_STORAGE_FULL

		case PM_EVT_ERROR_UNEXPECTED:
			// Assert.
			APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
			break; // PM_EVT_ERROR_UNEXPECTED

		case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
			break; // PM_EVT_PEER_DATA_UPDATE_SUCCEEDED

		case PM_EVT_PEER_DATA_UPDATE_FAILED:
			// Assert.
			APP_ERROR_CHECK_BOOL(false);
			break; // PM_EVT_PEER_DATA_UPDATE_FAILED

		case PM_EVT_PEER_DELETE_SUCCEEDED:
			break; // PM_EVT_PEER_DELETE_SUCCEEDED

		case PM_EVT_PEER_DELETE_FAILED:
			// Assert.
			APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
			break; // PM_EVT_PEER_DELETE_FAILED

		case PM_EVT_PEERS_DELETE_SUCCEEDED:
			advertising_start();
			break; // PM_EVT_PEERS_DELETE_SUCCEEDED

		case PM_EVT_PEERS_DELETE_FAILED:
			// Assert.
			APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
			break; // PM_EVT_PEERS_DELETE_FAILED

		case PM_EVT_LOCAL_DB_CACHE_APPLIED:
			break; // PM_EVT_LOCAL_DB_CACHE_APPLIED

		case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
			// The local database has likely changed, send service changed indications.
			pm_local_database_has_changed();
			break; // PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED

		case PM_EVT_SERVICE_CHANGED_IND_SENT:
			break; // PM_EVT_SERVICE_CHANGED_IND_SENT

		case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
			break; // PM_EVT_SERVICE_CHANGED_IND_SENT

		default:
			// No implementation needed.
			break;
	}
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error	 Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
	uint32_t			   err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params				   = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle	   = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail			   = false;
	cp_init.evt_handler					   = on_conn_params_evt;
	cp_init.error_handler				   = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are 
 *   passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	switch (ble_adv_evt)
	{
		case BLE_ADV_EVT_FAST:
		wowlMainSetStatus(WOWL_STATUS_ADVERTISING);
		break;

		case BLE_ADV_EVT_IDLE:
		wowlMainClearStatus(WOWL_STATUS_ADVERTISING);
		break;

		default:
		break;
	}
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt	 Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
	uint32_t err_code = NRF_SUCCESS;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_EVT_TX_COMPLETE: {
			const uint8_t 
				cnt = p_ble_evt->evt.common_evt.params.tx_complete.count;
			assert(nTxPending >= cnt);
			if (nTxPending >= cnt) {
				nTxPending -= cnt;
			} else {
				nTxPending = 0u;
			}
		}
		break;	
			
		case BLE_GAP_EVT_DISCONNECTED:
		// Indicate the change in the status word:
		wowlMainClearStatus(WOWL_STATUS_CONNECTED);
		// Clear the pending Tx count.
		nTxPending = 0u;
		break; // BLE_GAP_EVT_DISCONNECTED

		case BLE_GAP_EVT_CONNECTED:
		// Indicate the change in the status word:
		wowlMainSetStatus(WOWL_STATUS_CONNECTED);
		// Clear system attributes:
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
		APP_ERROR_CHECK(err_code);	
		break; // BLE_GAP_EVT_CONNECTED
		
		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		// No system attributes have been stored.
		err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
		APP_ERROR_CHECK(err_code);
		break;				

		case BLE_GATTC_EVT_TIMEOUT:
		// Disconnect on GATT Client timeout event.
		err_code = sd_ble_gap_disconnect(
				p_ble_evt->evt.gattc_evt.conn_handle,
				BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION
		);
		APP_ERROR_CHECK(err_code);
		break; // BLE_GATTC_EVT_TIMEOUT

		case BLE_GATTS_EVT_TIMEOUT:
		// Disconnect on GATT Server timeout event.
		err_code = sd_ble_gap_disconnect(
				p_ble_evt->evt.gatts_evt.conn_handle,
				BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION
		);
		APP_ERROR_CHECK(err_code);
		break; // BLE_GATTS_EVT_TIMEOUT

		case BLE_EVT_USER_MEM_REQUEST: {
			static ble_user_mem_block_t memBlock;
			
			memBlock.len = sizeof(writeBuffer);
			memBlock.p_mem = writeBuffer;
			err_code = sd_ble_user_mem_reply(
					p_ble_evt->evt.gattc_evt.conn_handle,
					&memBlock
			);
			APP_ERROR_CHECK(err_code);
		}
		break; // BLE_EVT_USER_MEM_REQUEST

		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST: {
			ble_gatts_evt_rw_authorize_request_t req;

			req = p_ble_evt->evt.gatts_evt.params.authorize_request;

			if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID) {
				if (   (req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)
					|| (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
					|| (req.request.write.op 
							== BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)) {
					ble_gatts_rw_authorize_reply_params_t auth_reply;

					// Initialize the reply object:
					memset(&auth_reply, 0, sizeof(auth_reply));
					if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE) {
						auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
						// Allow an attibute update:
						auth_reply.params.write.update = 1;
					} else {
						auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
					}
					// The request is successful:
					auth_reply.params.write.gatt_status
							= BLE_GATT_STATUS_SUCCESS;
							
					// Transmit to the client.
					err_code = sd_ble_gatts_rw_authorize_reply(
							p_ble_evt->evt.gatts_evt.conn_handle,
							&auth_reply
					);
					APP_ERROR_CHECK(err_code);
				}
			}
		} break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
		case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
		// The client has requested a change in the MTU.  Reply with our
		//  desired value.
		err_code = sd_ble_gatts_exchange_mtu_reply(
				p_ble_evt->evt.gatts_evt.conn_handle,
				NRF_BLE_MAX_MTU_SIZE
		);
		APP_ERROR_CHECK(err_code);
		break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

		default:
		// No implementation needed.
		break;
	}
}


/**@brief Function for dispatching a BLE stack event to all modules with a 
 *   BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler
 *  after a BLE stack event has been received.
 *
 * @param[in] p_ble_evt	 Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	// The Connection state module has to be fed BLE events in order to 
	//  function correctly.  Remember to call ble_conn_state_on_ble_evt 
	//  before calling any ble_conns_state_* functions.
	ble_conn_state_on_ble_evt(p_ble_evt);
	pm_on_ble_evt(p_ble_evt);
	ble_conn_params_on_ble_evt(p_ble_evt);
	on_ble_evt(p_ble_evt);
	ble_advertising_on_ble_evt(p_ble_evt);
	
	// Pass the event to the CHA service:
	wowlServiceEventHandler(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler 
 *  after a system event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
	// Dispatch the system event to the fstorage module, where it will be
	// dispatched to the Flash Data Storage (FDS) module.
	fs_sys_event_handler(sys_evt);

	// Dispatch to the Advertising module last, since it will check if there 
	//  are any pending flash operations in fstorage. Let fstorage process
	//  system events first, so that it can report correctly to the 
	//  Advertising module.
	ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
	uint32_t err_code;

//	  nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
	static const nrf_clock_lf_cfg_t clock_lf_cfg = {
		.source = NRF_CLOCK_LF_SRC_XTAL,
		.rc_ctiv = 0,
		.rc_temp_ctiv = 0,
		.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
	};
	
	// This delay appears to be necessary for the SoC to be able to 
	//  reliably start its LF XTAL.
	// In debug, 100 usec failed but 250 worked.  Setting to 500.
	WAIT_USEC(500U);

	// Initialize the SoftDevice handler module.
	SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	ble_enable_params_t ble_enable_params;
	err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
													PERIPHERAL_LINK_COUNT,
													&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	// Check the ram settings against the used number of links
	CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

	// Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
	ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
	err_code = softdevice_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *						   persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
	ble_gap_sec_params_t sec_param;
	ret_code_t			 err_code;

	err_code = pm_init();
	APP_ERROR_CHECK(err_code);

	if (erase_bonds)
	{
		err_code = pm_peers_delete();
		APP_ERROR_CHECK(err_code);
	}

	memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

	// Security parameters to be used for all security procedures.
	sec_param.bond			 = SEC_PARAM_BOND;
	sec_param.mitm			 = SEC_PARAM_MITM;
	sec_param.lesc			 = SEC_PARAM_LESC;
	sec_param.keypress		 = SEC_PARAM_KEYPRESS;
	sec_param.io_caps		 = SEC_PARAM_IO_CAPABILITIES;
	sec_param.oob			 = SEC_PARAM_OOB;
	sec_param.min_key_size	 = SEC_PARAM_MIN_KEY_SIZE;
	sec_param.max_key_size	 = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc	 = 1;
	sec_param.kdist_own.id	 = 1;
	sec_param.kdist_peer.enc = 1;
	sec_param.kdist_peer.id	 = 1;

	err_code = pm_sec_params_set(&sec_param);
	APP_ERROR_CHECK(err_code);

	err_code = pm_register(pm_evt_handler);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
	uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);

	APP_ERROR_CHECK(err_code);
}

