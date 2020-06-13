/*******************************************************************************
 * Copyright (C) Gallium Studio LLC. All rights reserved.
 *
 * This program is open source software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Alternatively, this program may be distributed and modified under the
 * terms of Gallium Studio LLC commercial licenses, which expressly supersede
 * the GNU General Public License and are specifically designed for licensees
 * interested in retaining the proprietary status of their code.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Contact information:
 * Website - https://www.galliumstudio.com
 * Source repository - https://github.com/galliumstudio
 * Email - admin@galliumstudio.com
 ******************************************************************************/

#include "app_hsmn.h"
#include "fw_log.h"
#include "fw_assert.h"
#include "SimpleActInterface.h"
#include "SimpleAct.h"
#include "WifiInterface.h"
#include "SensorAccelGyroInterface.h"
#include "GpioInInterface.h"

FW_DEFINE_THIS_FILE("SimpleAct.cpp")

const char * INIT_MAC_ADDRESS = "FF:FF:FF:FF:FF:FF";

namespace APP {

#undef ADD_EVT
#define ADD_EVT(e_) #e_,

static char const * const timerEvtName[] = {
    "SIMPLE_ACT_TIMER_EVT_START",
    SIMPLE_ACT_TIMER_EVT
};

static char const * const internalEvtName[] = {
    "SIMPLE_ACT_INTERNAL_EVT_START",
    SIMPLE_ACT_INTERNAL_EVT
};

static char const * const interfaceEvtName[] = {
    "SIMPLE_ACT_INTERFACE_EVT_START",
    SIMPLE_ACT_INTERFACE_EVT
};

SimpleAct::SimpleAct() :
    Active((QStateHandler)&SimpleAct::InitialPseudoState, SIMPLE_ACT, "SIMPLE_ACT"),
    m_stateTimer(GetHsm().GetHsmn(), STATE_TIMER),
	m_retryTimer(GetHsm().GetHsmn(), RETRY_TIMER), m_macAddressSet(false),
	m_sensorEnabled(false), m_shockEventSeq(0) {
	STRING_COPY(m_macAddress, INIT_MAC_ADDRESS, sizeof(m_macAddress));
    SET_EVT_NAME(SIMPLE_ACT);
}

QState SimpleAct::InitialPseudoState(SimpleAct * const me, QEvt const * const e) {
    (void)e;
    return Q_TRAN(&SimpleAct::Root);
}

QState SimpleAct::Root(SimpleAct * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            EVENT(e);
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            EVENT(e);
            return Q_HANDLED();
        }
        case Q_INIT_SIG: {
            return Q_TRAN(&SimpleAct::Stopped);
        }
        case SIMPLE_ACT_START_REQ: {
            EVENT(e);
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new SimpleActStartCfm(req.GetFrom(), GET_HSMN(), req.GetSeq(), ERROR_STATE, GET_HSMN());
            Fw::Post(evt);
            return Q_HANDLED();
        }
        case SIMPLE_ACT_STOP_REQ: {
            EVENT(e);
            Evt const &req = EVT_CAST(*e);
            me->GetHsm().SaveInSeq(req);
            return Q_TRAN(&SimpleAct::Stopping);
        }
    }
    return Q_SUPER(&QHsm::top);
}

QState SimpleAct::Stopped(SimpleAct * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            EVENT(e);
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            EVENT(e);
            return Q_HANDLED();
        }
        case SIMPLE_ACT_STOP_REQ: {
            EVENT(e);
            Evt const &req = EVT_CAST(*e);
            Evt *evt = new SimpleActStopCfm(req.GetFrom(), GET_HSMN(), req.GetSeq(), ERROR_SUCCESS);
            Fw::Post(evt);
            return Q_HANDLED();
        }
        case SIMPLE_ACT_START_REQ: {
            EVENT(e);
            Evt const &req = EVT_CAST(*e);
            me->GetHsm().SaveInSeq(req);
            return Q_TRAN(&SimpleAct::Starting);
        }
    }
    return Q_SUPER(&SimpleAct::Root);
}

QState SimpleAct::Starting(SimpleAct * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            EVENT(e);
            uint32_t timeout = SimpleActStartReq::TIMEOUT_MS;
            //FW_ASSERT(timeout > XxxStartReq::TIMEOUT_MS);
            FW_ASSERT(timeout > SensorAccelGyroOnReq::TIMEOUT_MS);
            me->m_stateTimer.Start(timeout);
            me->GetHsm().ResetOutSeq();
            //Evt *evt = new XxxStartReq(XXX, GET_HSMN(), GEN_SEQ());
            //me->GetHsm().SaveOutSeq(*evt);
            //Fw::Post(evt);
            // For testing, send DONE immediately. Do not use PostSync in entry action.
            //Evt *evt = new Evt(DONE, GET_HSMN());
            //Fw::Post(evt);
            Evt* evt = new SensorAccelGyroOnReq(IKS01A1_ACCEL_GYRO, GET_HSMN(), GEN_SEQ(), NULL);
			me->GetHsm().SaveOutSeq(*evt);
			Fw::Post(evt);
			evt = new WifiMacAddressReq(WIFI, GET_HSMN(), GEN_SEQ());
			me->GetHsm().SaveOutSeq(*evt);
			Fw::Post(evt);
			evt = new GpioInStartReq(USER_BTN, GET_HSMN(), GEN_SEQ());
			me->GetHsm().SaveOutSeq(*evt);
			Fw::Post(evt);
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            EVENT(e);
            me->m_stateTimer.Stop();
            me->GetHsm().ClearInSeq();
            return Q_HANDLED();
        }
        /*
        case XXX_START_CFM: {
            EVENT(e);
            ErrorEvt const &cfm = ERROR_EVT_CAST(*e);
            bool allReceived;
            if (!me->GetHsm().HandleCfmRsp(cfm, allReceived)) {
                Evt *evt = new Failed(GET_HSMN(), cfm.GetError(), cfm.GetOrigin(), cfm.GetReason());
                me->PostSync(evt);
            } else if (allReceived) {
                Evt *evt = new Evt(DONE, GET_HSMN());
                me->PostSync(evt);
            }
            return Q_HANDLED();
        }
        */
        case WIFI_MAC_ADDRESS_CFM: {
        	EVENT(e);
        	WifiMacAddressCfm* macCfm = (WifiMacAddressCfm*)e;
        	STRING_COPY(me->m_macAddress, macCfm->getMacAddress(), sizeof(me->m_macAddress));
        	ErrorEvt const &cfm = ERROR_EVT_CAST(*e);
			bool allReceived;
			if (!me->GetHsm().HandleCfmRsp(cfm, allReceived)) {
				LOG("failed from sensor cfm");
				Evt *evt = new Failed(GET_HSMN(), cfm.GetError(), cfm.GetOrigin(), cfm.GetReason());
				me->PostSync(evt);
			} else if (allReceived) {
				LOG("sending done...");
				Evt *evt = new Evt(DONE, GET_HSMN());
				me->PostSync(evt);
			}
			LOG("so sad");
			me->m_macAddressSet = true;
			if(me->m_sensorEnabled) {
				Evt* evt = new Evt(DONE, GET_HSMN());
				me->PostSync(evt);
			}
			return Q_HANDLED();
        }
        case GPIO_IN_START_CFM:
        case SENSOR_ACCEL_GYRO_ON_CFM: {
			EVENT(e);
			ErrorEvt const &cfm = ERROR_EVT_CAST(*e);
			bool allReceived;
			if (!me->GetHsm().HandleCfmRsp(cfm, allReceived)) {
				LOG("failed from sensor cfm");
				Evt *evt = new Failed(GET_HSMN(), cfm.GetError(), cfm.GetOrigin(), cfm.GetReason());
				me->PostSync(evt);
			} else if (allReceived) {
				Evt *evt = new Evt(DONE, GET_HSMN());
				me->PostSync(evt);
			}
			me->m_sensorEnabled = true;
			if(me->m_macAddressSet) {
				Evt* evt = new Evt(DONE, GET_HSMN());
				me->PostSync(evt);
			}
			return Q_HANDLED();
		}
        case FAILED:
        case STATE_TIMER: {
            EVENT(e);
            Evt *evt;
            if (e->sig == FAILED) {
                ErrorEvt const &failed = ERROR_EVT_CAST(*e);
                evt = new SimpleActStartCfm(me->GetHsm().GetInHsmn(), GET_HSMN(), me->GetHsm().GetInSeq(),
                                            failed.GetError(), failed.GetOrigin(), failed.GetReason());
            } else {
                evt = new SimpleActStartCfm(me->GetHsm().GetInHsmn(), GET_HSMN(), me->GetHsm().GetInSeq(), ERROR_TIMEOUT, GET_HSMN());
            }
            Fw::Post(evt);
            return Q_TRAN(&SimpleAct::Stopping);
        }
        case DONE: {
            EVENT(e);
            Evt *evt = new SimpleActStartCfm(me->GetHsm().GetInHsmn(), GET_HSMN(), me->GetHsm().GetInSeq(), ERROR_SUCCESS);
            Fw::Post(evt);
            return Q_TRAN(&SimpleAct::Started);
        }
    }
    return Q_SUPER(&SimpleAct::Root);
}

QState SimpleAct::Stopping(SimpleAct * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            EVENT(e);
            uint32_t timeout = SimpleActStopReq::TIMEOUT_MS;
            //FW_ASSERT(timeout > XxxStopReq::TIMEOUT_MS);
            me->m_stateTimer.Start(timeout);
            me->GetHsm().ResetOutSeq();
            //Evt *evt = new XxxStopReq(XXX, GET_HSMN(), GEN_SEQ());
            //me->GetHsm().SaveOutSeq(*evt);
            //Fw::Post(evt);
            // For testing, send DONE immediately. Do not use PostSync in entry action.
            //Evt *evt = new Evt(DONE, GET_HSMN());
            //Fw::Post(evt);
            Evt* evt = new SensorAccelGyroOffReq(IKS01A1_ACCEL_GYRO, GET_HSMN(), GEN_SEQ());
			me->GetHsm().SaveOutSeq(*evt);
			Fw::Post(evt);
			evt = new GpioInStopReq(USER_BTN, GET_HSMN(), GEN_SEQ());
			me->GetHsm().SaveOutSeq(*evt);
			Fw::Post(evt);
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            EVENT(e);
            me->m_stateTimer.Stop();
            me->GetHsm().ClearInSeq();
            me->GetHsm().Recall();
            return Q_HANDLED();
        }
        case SIMPLE_ACT_STOP_REQ: {
            EVENT(e);
            me->GetHsm().Defer(e);
            return Q_HANDLED();
        }
        /*
        case XXX_STOP_CFM: {
            EVENT(e);
            ErrorEvt const &cfm = ERROR_EVT_CAST(*e);
            bool allReceived;
            if (!me->GetHsm().HandleCfmRsp(cfm, allReceived)) {
                Evt *evt = new Failed(GET_HSMN(), cfm.GetError(), cfm.GetOrigin(), cfm.GetReason());
                me->PostSync(evt);
            } else if (allReceived) {
                Evt *evt = new Evt(DONE, GET_HSMN());
                me->PostSync(evt);
            }
            return Q_HANDLED();
        }
        */
        case GPIO_IN_STOP_CFM:
        case SENSOR_ACCEL_GYRO_OFF_CFM: {
			EVENT(e);
			ErrorEvt const &cfm = ERROR_EVT_CAST(*e);
			bool allReceived;
			if (!me->GetHsm().HandleCfmRsp(cfm, allReceived)) {
				Evt *evt = new Failed(GET_HSMN(), cfm.GetError(), cfm.GetOrigin(), cfm.GetReason());
				me->PostSync(evt);
			} else if (allReceived) {
				Evt *evt = new Evt(DONE, GET_HSMN());
				me->PostSync(evt);
			}
			return Q_HANDLED();
		}
        case FAILED:
        case STATE_TIMER: {
            EVENT(e);
            FW_ASSERT(0);
            // Will not reach here.
            return Q_HANDLED();
        }
        case DONE: {
            EVENT(e);
            Evt *evt = new SimpleActStopCfm(me->GetHsm().GetInHsmn(), GET_HSMN(), me->GetHsm().GetInSeq(), ERROR_SUCCESS);
            Fw::Post(evt);
            return Q_TRAN(&SimpleAct::Stopped);
        }
    }
    return Q_SUPER(&SimpleAct::Root);
}

QState SimpleAct::Started(SimpleAct * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            EVENT(e);
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            EVENT(e);
            // TODO: turn LED OFF
            return Q_HANDLED();
        }
        case Q_INIT_SIG: {
        	EVENT(e);
        	return Q_TRAN(&SimpleAct::Disconnected);
        }
//        case WIFI_UP: {
//        	EVENT(e);
//        	return Q_TRAN(&SimpleAct::Started);
//        }
    }
    return Q_SUPER(&SimpleAct::Root);
}

QState SimpleAct::Disconnected(SimpleAct * const me, QEvt const * const e) {
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			EVENT(e);
			me->m_retryTimer.Start(5000);
			Evt* evt = new WifiConnectReq(WIFI, GET_HSMN(), GEN_SEQ(), "192.168.0.15", 60002);
			Fw::Post(evt);
			return Q_HANDLED();
		}
		case Q_EXIT_SIG: {
			EVENT(e);
			me->m_retryTimer.Stop();
			return Q_HANDLED();
		}
		case RETRY_TIMER: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Disconnected);
		}
		case WIFI_CONNECT_CFM: {
			EVENT(e);
			Evt * evt = new ShockSensorHubConnected(GET_HSMN(), GET_HSMN(), GEN_SEQ());
			Fw::Post(evt);
			return Q_HANDLED();
		}
		case SHOCK_SENSOR_HUB_CONNECTED: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Connected);
		}
	}
	return Q_SUPER(&SimpleAct::Started);
}

QState SimpleAct::Connected(SimpleAct * const me, QEvt const * const e) {
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			EVENT(e);
			//me->m_stateTimer.Start(15000);
			return Q_HANDLED();
		}
		case Q_EXIT_SIG: {
			EVENT(e);
			return Q_HANDLED();
		}
		case Q_INIT_SIG: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Unregistered);
		}
		case WIFI_DISCONNECT_CFM: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Disconnected);
		}
	}
	return Q_SUPER(&SimpleAct::Started);
}

QState SimpleAct::Unregistered(SimpleAct * const me, QEvt const * const e) {
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			EVENT(e);
			LOG("fucking kill me!!!!!!!");
			//me->m_retryTimer.Start(5000);
			me->m_stateTimer.Start(5000);
			char handshake[100];
			snprintf(handshake, sizeof(handshake), "SENSOR-CONNECT**%s", me->getMacAddress());
			LOG("HANDSHAKE: %s", handshake);
			Evt * evt = new WifiSendReq(WIFI, GET_HSMN(), GEN_SEQ(), handshake);
			Fw::Post(evt);
			return Q_HANDLED();
		}
		case Q_EXIT_SIG: {
			EVENT(e);
			me->m_stateTimer.Stop();
			return Q_HANDLED();
		}
		case STATE_TIMER: {
			EVENT(e);
			Evt* evt = new WifiDisconnectReq(WIFI, GET_HSMN(), GEN_SEQ());
			Fw::Post(evt);
			return Q_HANDLED();
		}
		case SHOCK_SENSOR_HANDSHAKE_CFM: {
			EVENT(e);
			me->m_retryTimer.Stop();
			return Q_TRAN(&SimpleAct::Registered);
		}
		case RETRY_TIMER: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Disconnected);
		}
	}
	return Q_SUPER(&SimpleAct::Connected);
}

QState SimpleAct::Registered(SimpleAct * const me, QEvt const * const e) {
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			EVENT(e);
			me->m_stateTimer.Start(15000);
			return Q_HANDLED();
		}
		case Q_EXIT_SIG: {
			EVENT(e);
			return Q_HANDLED();
		}
		case Q_INIT_SIG: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Listening);
		}
		case SHOCK_SENSOR_TEST_EVENT: {
			EVENT(e);
			char msg[100];
			snprintf(msg, sizeof(msg), "SENSOR-TEST-EVENT**%s", me->getMacAddress());
			Evt* evt = new WifiSendReq(WIFI, GET_HSMN(), GEN_SEQ(), msg);
			Fw::Post(evt);
			return Q_HANDLED();
		}
		case GPIO_IN_HOLD_IND: {
			EVENT(e);
			Evt* evt = new ShockSensorTestEvent(GET_HSMN(), GET_HSMN(), GEN_SEQ());
			me->PostSync(evt);
			return Q_HANDLED();
		}
		case STATE_TIMER: {
			EVENT(e);
			char msg[100];
			snprintf(msg, sizeof(msg), "SENSOR-HEARTBEAT");
			Evt* evt = new WifiSendReq(WIFI, GET_HSMN(), GEN_SEQ(), msg);
			Fw::Post(evt);
			me->m_stateTimer.Start(15000);
			return Q_HANDLED();
		}
	}
	return Q_SUPER(&SimpleAct::Connected);
}

QState SimpleAct::Listening(SimpleAct * const me, QEvt const * const e) {
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			EVENT(e);
			// TODO: turn LED OFF
			return Q_HANDLED();
		}
		case Q_EXIT_SIG: {
			EVENT(e);
			return Q_HANDLED();
		}
		case SHOCK_SENSOR_SHOCK_EVENT: {
			EVENT(e);
			char msg[100];
			snprintf(msg, sizeof(msg), "SENSOR-SHOCK-EVENT**%s", me->getMacAddress());
			me->m_shockEventSeq = GEN_SEQ();
			Evt* evt = new WifiSendReq(WIFI, GET_HSMN(), me->m_shockEventSeq, msg);
			Fw::Post(evt);
			return Q_HANDLED();
		}
		case WIFI_SEND_CFM: {
			EVENT(e);
			WifiSendCfm const &cfm = static_cast<WifiSendCfm const &>(*e);
			if(cfm.GetSeq() == me->m_shockEventSeq) {
				Evt* evt = new Evt(TRIGGERED, GET_HSMN());
				me->PostSync(evt);
			}
			return Q_HANDLED();
		}
		case TRIGGERED: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Triggered);
		}
		case GPIO_IN_PULSE_IND: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Triggered);
		}
	}
	return Q_SUPER(&SimpleAct::Registered);
}

QState SimpleAct::Triggered(SimpleAct * const me, QEvt const * const e) {
	switch(e->sig) {
		case Q_ENTRY_SIG: {
			EVENT(e);
			// TODO: turn LED ON
			return Q_HANDLED();
		}
		case Q_EXIT_SIG: {
			EVENT(e);
			return Q_HANDLED();
		}
		case SHOCK_SENSOR_RESET: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Listening);
		}
		case GPIO_IN_PULSE_IND: {
			EVENT(e);
			return Q_TRAN(&SimpleAct::Listening);
		}
	}
	return Q_SUPER(&SimpleAct::Registered);
}

/*
QState SimpleAct::MyState(SimpleAct * const me, QEvt const * const e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
            EVENT(e);
            return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
            EVENT(e);
            return Q_HANDLED();
        }
        case Q_INIT_SIG: {
            return Q_TRAN(&SimpleAct::SubState);
        }
    }
    return Q_SUPER(&SimpleAct::SuperState);
}
*/

} // namespace APP
