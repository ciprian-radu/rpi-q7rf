#include "rpi-q7rf.h"
#include "cc1100_raspi.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <getopt.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

namespace q7rf {

static const char *TAG = "q7rf.switch";

static const char *Q7RF_PREAMBLE_DATA = "111000111";
static const char *Q7RF_ZERO_BIT_DATA = "011";
static const char *Q7RF_ONE_BIT_DATA = "001";
static const char *Q7RF_GAP_DATA = "000";

static const uint8_t Q7RF_MSG_CMD_PAIR = 0x00;
static const uint8_t Q7RF_MSG_CMD_TURN_ON_HEATING = 0xFF;
static const uint8_t Q7RF_MSG_CMD_TURN_OFF_HEATING = 0x0F;

static const uint8_t CMD_SRES = 0x30;
static const uint8_t CMD_STX = 0x35;
static const uint8_t CMD_SIDLE = 0x36;
static const uint8_t CMD_SFRX = 0x3a;
static const uint8_t CMD_SFTX = 0x3b;

static const uint8_t REG_PARTNUM = 0xf0;
static const uint8_t REG_VERSION = 0xf1;

static const uint8_t CREG_FIFOTHR = 0x03;
static const uint8_t CREG_PKTLEN = 0x06;
static const uint8_t CREG_PKTCTRL1 = 0x07;
static const uint8_t CREG_PKTCTRL0 = 0x08;
static const uint8_t CREG_FREQ2 = 0x0d;
static const uint8_t CREG_FREQ1 = 0x0e;
static const uint8_t CREG_FREQ0 = 0x0f;
static const uint8_t CREG_MDMCFG4 = 0x10;
static const uint8_t CREG_MDMCFG3 = 0x11;
static const uint8_t CREG_MDMCFG2 = 0x12;
static const uint8_t CREG_MDMCFG1 = 0x13;
static const uint8_t CREG_MDMCFG0 = 0x14;
static const uint8_t CREG_MCSM0 = 0x18;
static const uint8_t CREG_FOCCFG = 0x19;
static const uint8_t CREG_FREND0 = 0x22;

static const uint8_t SREG_MARCSTATE = 0xf5;

static const uint8_t CREG_PATABLE_BURST_WRITE = 0x7e;
static const uint8_t CREG_PATABLE_BURST_READ = 0xfe;
static const uint8_t CFIFO_TX_BURST = 0x7f;

static const uint8_t MARCSTATE_TX = 0x13;
static const uint8_t MARCSTATE_TX_END = 0x14;
static const uint8_t MARCSTATE_RXTX_SWITCH = 0x15;

static const uint8_t MSG_SEND_ERRORS_RESET_LIMIT = 3;

/* Each symbol takes 220us. Computherm/Delta Q7RF uses PWM modulation.
   Every data bit is encoded as 3 bit inside the buffer.
   001 = 1, 011 = 0, 111000111 = preamble */
static const uint8_t Q7RF_REG_CONFIG[] = {
    CREG_FIFOTHR,  0x00,  // TX FIFO length = 61, others default
    CREG_PKTLEN,   0x3d,  // 61 byte packets
    CREG_PKTCTRL1, 0x00,  // Disable RSSI/LQ payload sending, no address check
    CREG_PKTCTRL0, 0x01,  // variable packet length, no CRC calculation
    CREG_FREQ2,    0x21,  // FREQ2, FREQ=0x216544 => 2.188.612 * (26 MHz OSC / 2^16) ~= 868.285 MHz
    CREG_FREQ1,    0x65,  // ^FREQ1
    CREG_FREQ0,    0x44,  // ^FREQ0
    CREG_MDMCFG4,  0xf7,  // baud exponent = 7 (lower 4 bits)
    CREG_MDMCFG3,  0x6B,  // baud mantissa = 107 -> (((256+107) * 2^7)/2^28) * 26 MHz OSC = 4.5 kBaud
    CREG_MDMCFG2,  0x30,  // DC filter on, ASK/OOK modulation, no manchester coding, no preamble/sync
    CREG_MDMCFG1,  0x00,  // no FEC, channel spacing exponent = 0 (last two bit)
    CREG_MDMCFG0,  0xf8,  // channel spacing mantissa = 248 -> 6.000.000 / 2^18 * (256 + 248) * 2^0 ~= 50kHz
    CREG_MCSM0,    0x10,  // autocalibrate synthesizer when switching from IDLE to RX/TX state
    CREG_FOCCFG,   0x00,  // ASK/OOK has no frequency offset compensation
    CREG_FREND0,   0x11   // ASK/OOK PATABLE (power level) settings = up to index 1
};

// 0xc0 = +12dB max power setting
static const uint8_t Q7RF_PA_TABLE[] = {0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

unsigned long elapsed(unsigned long since, unsigned long now) {
  if (since > now) {
    // millis() overflows every ~50 days
    return (ULONG_MAX - since) + now;
  } else {
    return now - since;
  }
}

uint8_t state_to_msg(bool state) { return state ? MSG_HEAT_ON : MSG_HEAT_OFF; }

void encode_bits(uint16_t byte, uint8_t pad_to_length, char **dest) {
  char binary[9];
  //itoa(byte, binary, 2);
  strcpy(binary, std::bitset<16>(byte).to_string().c_str());
  int binary_len = strlen(binary);

  if (binary_len < pad_to_length) {
    for (int p = 0; p < pad_to_length - binary_len; p++) {
      strncpy(*dest, Q7RF_ZERO_BIT_DATA, strlen(Q7RF_ZERO_BIT_DATA));
      *dest += strlen(Q7RF_ZERO_BIT_DATA);
    }
  }

  for (int b = 0; b < binary_len; b++) {
    strncpy(*dest, binary[b] == '1' ? Q7RF_ONE_BIT_DATA : Q7RF_ZERO_BIT_DATA, strlen(Q7RF_ONE_BIT_DATA));
    *dest += strlen(Q7RF_ZERO_BIT_DATA);
  }
}

void compile_msg(uint16_t device_id, uint8_t cmd, uint8_t *msg) {
  char binary_msg[360];
  char *cursor = binary_msg;

  // Preamble
  char *preamble_start = cursor;
  strncpy(cursor, Q7RF_PREAMBLE_DATA, strlen(Q7RF_PREAMBLE_DATA));
  cursor += strlen(Q7RF_PREAMBLE_DATA);

  char *payload_start = cursor;

  // Command
  encode_bits(device_id, 16, &cursor);
  encode_bits(8, 4, &cursor);
  encode_bits(cmd, 8, &cursor);

  // Repeat the command once more
  strncpy(cursor, payload_start, cursor - payload_start);
  cursor += cursor - payload_start;

  // Add a gap
  strncpy(cursor, Q7RF_GAP_DATA, strlen(Q7RF_GAP_DATA));
  cursor += strlen(Q7RF_GAP_DATA);

  // Repeat the whole burst
  strncpy(cursor, preamble_start, cursor - preamble_start);
  cursor += cursor - preamble_start;

  // Convert msg to bytes
  cursor = binary_msg;  // Reset cursor
  uint8_t *cursor_msg = msg;
  char binary_byte[9];
  binary_byte[8] = '\0';
  for (int b = 0; b < 45; b++) {
    strncpy(binary_byte, cursor, 8);
    cursor += 8;
    *cursor_msg = strtoul(binary_byte, 0, 2);
    cursor_msg++;
  }

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  // Assemble debug print
  char debug[91];
  cursor = debug;
  cursor_msg = msg;
  for (int b = 0; b < 45; b++) {
    sprintf(cursor, "%x", *cursor_msg);
    cursor += 2;
    cursor_msg++;
  }
  printf("Encoded msg: 0x%02x as 0x%s", cmd, debug);
#endif
}

bool Q7RFSwitch::reset_cc() {
  // Chip reset sequence. CS wiggle (CC1101 manual page 45)

  // TODO
  //this->disable();
  //delayMicroseconds(5);
  //this->enable();
  //delayMicroseconds(10);
  //this->disable();
  //delayMicroseconds(41);

  this->send_cc_cmd(CMD_SRES);
  printf("Issued CC1101 reset sequence.");

  // Read part number and version
  uint8_t partnum;
  this->read_cc_register(REG_PARTNUM, &partnum);

  uint8_t version;
  this->read_cc_register(REG_VERSION, &version);

  printf("CC1101 found with partnum: %02x and version: %02x", partnum, version);

  // Setup config registers
  uint8_t verify_value;
  for (int i = 0; i < sizeof(Q7RF_REG_CONFIG); i += 2) {
    this->write_cc_config_register(Q7RF_REG_CONFIG[i], Q7RF_REG_CONFIG[i + 1]);
    this->read_cc_config_register(Q7RF_REG_CONFIG[i], &verify_value);
    if (verify_value != Q7RF_REG_CONFIG[i + 1]) {
      printf("Failed to write CC1101 config register. reg: %02x write: %02x read: %02x", Q7RF_REG_CONFIG[i],
               Q7RF_REG_CONFIG[i + 1], verify_value);
      return false;
    }
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
    printf("Written CC1101 config register. reg: %02x value: %02x", Q7RF_REG_CONFIG[i], Q7RF_REG_CONFIG[i + 1]);
#endif
  }

  // Write PATable
  uint8_t pa_table[sizeof(Q7RF_PA_TABLE)];
  for (int i = 0; i < sizeof(Q7RF_PA_TABLE); i++)
    pa_table[i] = Q7RF_PA_TABLE[i];

  //TODO
  //this->enable();
  //this->write_byte(CREG_PATABLE_BURST_WRITE);
  //this->write_array(pa_table, sizeof(Q7RF_PA_TABLE));
  //this->disable();

  //this->enable();
  //this->write_byte(CREG_PATABLE_BURST_READ);
  //this->read_array(pa_table, sizeof(Q7RF_PA_TABLE));
  //this->disable();

  for (int i = 0; i < sizeof(Q7RF_PA_TABLE); i++) {
    if (pa_table[i] != Q7RF_PA_TABLE[i]) {
      printf("Failed to write CC1101 PATABLE.");
      return false;
    }
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
    printf("Written CC1101 PATABLE[%d]: %02x", i, Q7RF_PA_TABLE[i]);
#endif
  }

  return true;
}

void Q7RFSwitch::send_cc_cmd(uint8_t cmd) {
  //TODO
  //this->enable();
  //this->transfer_byte(cmd);
  //this->disable();
}

void Q7RFSwitch::read_cc_register(uint8_t reg, uint8_t *value) {
  //TODO
  //this->enable();
  //this->transfer_byte(reg);
  //*value = this->transfer_byte(0);
  //this->disable();
}

void Q7RFSwitch::read_cc_config_register(uint8_t reg, uint8_t *value) { this->read_cc_register(reg + 0x80, value); }

void Q7RFSwitch::write_cc_register(uint8_t reg, uint8_t *value, size_t length) {
  //TODO
  //this->enable();
  //this->transfer_byte(reg);
  //this->transfer_array(value, length);
  //this->disable();
}

void Q7RFSwitch::write_cc_config_register(uint8_t reg, uint8_t value) {
  uint8_t arr[1] = {value};
  //TODO
  //this->write_cc_register(reg, arr, 1);
}

bool Q7RFSwitch::send_cc_data(const uint8_t *data, size_t length) {
  uint8_t buffer[length];
  for (int i = 0; i < length; i++) {
    buffer[i] = *data;
    data++;
  }

  this->send_cc_cmd(CMD_SIDLE);
  this->send_cc_cmd(CMD_SFRX);
  this->send_cc_cmd(CMD_SFTX);

  //TODO
  //this->enable();
  //this->write_byte(CFIFO_TX_BURST);
  //this->write_array(buffer, length);
  //this->disable();

  this->send_cc_cmd(CMD_STX);

  uint8_t state;
  this->read_cc_register(SREG_MARCSTATE, &state);
  state &= 0x1f;
  if (state != MARCSTATE_TX && state != MARCSTATE_TX_END && state != MARCSTATE_RXTX_SWITCH) {
    printf("CC1101 in invalid state after sending, returning to idle. State: 0x%02x", state);
    this->send_cc_cmd(CMD_SIDLE);
    return false;
  }

  return true;
}

bool Q7RFSwitch::send_msg(uint8_t msg) {
  if (msg == MSG_NONE)
    return false;

  bool result = false;
  const char *text_msg = NULL;
  switch (msg) {
    case MSG_HEAT_ON:
      result = this->send_cc_data(this->msg_heat_on_, sizeof(this->msg_heat_on_));
      text_msg = "HEAT ON";
      break;
    case MSG_HEAT_OFF:
      result = this->send_cc_data(this->msg_heat_off_, sizeof(this->msg_heat_off_));
      text_msg = "HEAT OFF";
      break;
    case MSG_PAIR:
      result = this->send_cc_data(this->msg_pair_, sizeof(this->msg_pair_));
      text_msg = "PAIR";
      break;
  }

  if (text_msg) {
    printf("Sent message: %s", text_msg);
  }

  if (result) {
    this->last_msg_time_ = millis();
  } else {
    this->msg_errors_++;
    if (this->msg_errors_ >= MSG_SEND_ERRORS_RESET_LIMIT) {
      printf("Multiple message send errors occured, forcing CC1101 reset.");
      this->reset_cc();
      this->msg_errors_ = 0;
    }
  }

  return result;
}

long Q7RFSwitch::millis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void Q7RFSwitch::set_state(bool state) {
  this->state_ = state;
  //TODO
  //this->publish_state(state);
}

void Q7RFSwitch::setup() {
  // Revert switch to off state
  this->set_state(false);

  // Compile messages
  compile_msg(this->q7rf_device_id_, Q7RF_MSG_CMD_PAIR, this->msg_pair_);
  compile_msg(this->q7rf_device_id_, Q7RF_MSG_CMD_TURN_ON_HEATING, this->msg_heat_on_);
  compile_msg(this->q7rf_device_id_, Q7RF_MSG_CMD_TURN_OFF_HEATING, this->msg_heat_off_);

  // Register the pairing service
  //TODO
  // register_service(&Q7RFSwitch::on_pairing, "q7rf_pair");

  //TODO
  //this->spi_setup();
  if (this->reset_cc()) {
    printf("CC1101 initialized.");
  } else {
    printf("Failed to reset CC1101 modem. Check connection.");
    return;
  }

  this->initialized_ = true;
}

void Q7RFSwitch::on_pairing() {
  if (this->initialized_) {
    this->pending_msg_ = MSG_PAIR;
    printf("Enqueued pairing.");
  }
}

void Q7RFSwitch::write_state(bool state) {
  if (this->initialized_) {
    if (state) {
      this->last_turn_on_time_ = millis();
    }

    if (this->state_ != state) {
      this->set_state(state);
      this->pending_msg_ = state_to_msg(state);
    }
  }
}

void Q7RFSwitch::dump_config() {
  printf("Q7RF:");
  //printf("  CC1101 CS Pin: ", this->cs_);
  printf("  Q7RF Device ID: 0x%04x", this->q7rf_device_id_);
  printf("  Q7RF Resend interval: %d ms", this->q7rf_resend_interval_);
  printf("  Q7RF Turn on watchdog interval: %d ms", this->q7rf_turn_on_watchdog_interval_);
}

void Q7RFSwitch::update() {
  if (this->initialized_) {
    if (this->pending_msg_ != MSG_NONE) {
      printf("Handling prioritized message.");
      // Send prioritized message
      this->send_msg(this->pending_msg_);
      this->pending_msg_ = MSG_NONE;
    } else {
      unsigned long now = millis();

      if (this->state_ && this->q7rf_turn_on_watchdog_interval_ > 0 &&
          elapsed(this->last_turn_on_time_, now) > this->q7rf_turn_on_watchdog_interval_) {
        printf("Turn on watch dog triggered, turning off furnace.");
        this->set_state(false);
        this->send_msg(MSG_HEAT_OFF);
        return;
      }

      // Check if we have to resend current state by now
      if (elapsed(this->last_msg_time_, now) > this->q7rf_resend_interval_) {
        printf("Resending last state.");
        uint8_t msg = state_to_msg(this->state_);
        this->send_msg(msg);
      }
    }
  }
}

void Q7RFSwitch::set_q7rf_device_id(uint16_t id) { this->q7rf_device_id_ = id; }

void Q7RFSwitch::set_q7rf_resend_interval(uint32_t interval) { this->q7rf_resend_interval_ = interval; }

void Q7RFSwitch::set_q7rf_turn_on_watchdog_interval(uint32_t interval) {
  this->q7rf_turn_on_watchdog_interval_ = interval;
}

}  // namespace q7rf

#define PACKAGE    "CC1100 SW"
#define VERSION_SW "0.9.6"

#define INTERVAL_1S_TIMER 1000


//--------------------------[Global CC1100 variables]--------------------------
uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
uint8_t rx_addr,sender,lqi;
 int8_t rssi_dbm;

int cc1100_freq_select, cc1100_mode_select, cc1100_channel_select;

uint32_t prev_millis_1s_timer = 0;

uint8_t cc1100_debug = 0;								//set CC1100 lib in no-debug output mode
uint8_t tx_retries = 1;
uint8_t rx_demo_addr = 3;
int interval = 1000;

CC1100 cc1100;

//-------------------------- [End] --------------------------

void print_help(int exval) {
	printf("%s,%s by CW\r\n", PACKAGE, VERSION_SW);
	printf("%s [-h] [-V] [-a My_Addr] [-r RxDemo_Addr] [-i Msg_Interval] [-t tx_retries] [-c channel] [-f frequency]\r\n", PACKAGE);
	printf("          [-m modulation]\n\r\n\r");
	printf("  -h              			print this help and exit\r\n");
	printf("  -V              			print version and exit\r\n\r\n");
	printf("  -v              			set verbose flag\r\n");
	printf("  -a my address [1-255] 		set my address\r\n\r\n");
	printf("  -r rx address [1-255] 	  	set RxDemo receiver address\r\n\r\n");
	printf("  -i interval ms[1-6000] 	  	sets message interval timing\r\n\r\n");
	printf("  -t tx_retries [0-255] 	  	sets message send retries\r\n\r\n");
	printf("  -c channel 	[1-255] 		set transmit channel\r\n");
	printf("  -f frequency  [315,434,868,915]  	set ISM band\r\n\r\n");
	printf("  -m modulation [1,38,100,250,500,4]	set modulation\r\n\r\n");

	exit(exval);
}


//|============================ Main ============================|
int main(int argc, char *argv[]) {
	//------------- command line option parser -------------------
	int opt;
	// no arguments given
	if(argc == 1) {
		fprintf(stderr, "This program needs arguments....\n\r\n\r");
		print_help(1);
	}

	while((opt = getopt(argc, argv, "hVva:r:i:t:c:f:m:")) != -1) {
		switch(opt) {
		case 'h':
			print_help(0);
			break;
		case 'V':
			printf("%s %s\n\n", PACKAGE, VERSION_SW);
			exit(0);
			break;
		case 'v':
			printf("%s: Verbose option is set `%c'\n", PACKAGE, optopt);
			cc1100_debug = 1;
			break;
		case 'a':
			My_addr = atoi (optarg);
			break;
		case 'r':
			rx_demo_addr = atoi (optarg);
			break;
		case 'i':
			interval = atoi (optarg);
			break;
		case 't':
			tx_retries = atoi (optarg);
			break;
		case 'c':
			cc1100_channel_select = atoi (optarg);
			break;
		case 'f':
			cc1100_freq_select = atoi (optarg);
			switch(cc1100_freq_select){
				case 315:
					cc1100_freq_select = 1;
					break;
				case 434:
					cc1100_freq_select = 2;
					break;
				case 868:
					cc1100_freq_select = 3;
					break;
				case 915:
					cc1100_freq_select = 4;
					break;
				}
			break;
			case 'm':
				cc1100_mode_select = atoi (optarg);

				switch(cc1100_mode_select){
				case 1:
					cc1100_mode_select = 1;
					break;
				case 38:
					cc1100_mode_select = 2;
					break;

				case 100:
					cc1100_mode_select = 3;
					break;
				case 250:
					cc1100_mode_select = 4;
					break;
				case 500:
					cc1100_mode_select = 5;
					break;
				case 4:
					cc1100_mode_select = 6;
					break;
				}
				break;
				case ':':
					fprintf(stderr, "%s: Error - Option `%c' needs a value\n\n", PACKAGE, optopt);
					print_help(1);
					break;
				case '?':
					fprintf(stderr, "%s: Error - No such option: `%c'\n\n", PACKAGE, optopt);
					print_help(1);
		}
	}

	// print all remaining options
	for(; optind < argc; optind++)
		printf("argument: %s\n", argv[optind]);

	//------------- welcome message ------------------------
	printf("Raspberry CC1101 SPI Library test\n");

	//------------- hardware setup ------------------------
	
	wiringPiSetup();			//setup wiringPi library

	cc1100.begin(My_addr);			//setup cc1000 RF IC
	cc1100.sidle();
	//cc1100.set_mode(0x04);                //set modulation mode 1 = GFSK_1_2_kb; 2 = GFSK_38_4_kb; 3 = GFSK_100_kb; 4 = MSK_250_kb; 5 = MSK_500_kb; 6 = OOK_4_8_kb
	//cc1100.set_ISM(0x03);                 //set ISM Band 1=315MHz; 2=433MHz; 3=868MHz; 4=915MHz
	//cc1100.set_channel(0x01);             //set channel
	cc1100.set_output_power_level(0);       //set PA level
	//cc1100.set_myaddr(0x05);
	
	cc1100.receive();

	cc1100.show_main_settings();             //shows setting debug messages to UART
	cc1100.show_register_settings();
	//------------------------- Main Loop ------------------------
	for (;;)
	{
		delay(1);                                           //delay to reduce system load

		if (millis() - prev_millis_1s_timer >= interval)    // one second update timer
		{
    			Rx_addr = rx_demo_addr;                     //receiver address

    			uint32_t time_stamp = millis();             //generate time stamp

    			Tx_fifo[3] = (uint8_t)(time_stamp >> 24);   //split 32-Bit timestamp to 4 byte array
    			Tx_fifo[4] = (uint8_t)(time_stamp >> 16);
    			Tx_fifo[5] = (uint8_t)(time_stamp >> 8);
    			Tx_fifo[6] = (uint8_t)(time_stamp);

    			Pktlen = 0x07;                              //set packet len to 0x13

    			uint8_t res = cc1100.sent_packet(My_addr, Rx_addr, Tx_fifo, Pktlen, tx_retries);
    			
    			if( res == 1)    //sents package over air. ACK is received via GPIO polling
    			{
    				printf("transmitted tx_timestamp: %ums \r\n\r\n", time_stamp);
				}
			prev_millis_1s_timer = millis();
  		}

	}
    printf("finished!\n");
    return 0;
}
//-------------------- END OF MAIN --------------------------------}
