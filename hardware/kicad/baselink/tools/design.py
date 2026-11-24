"""BaseLink controller board: complete circuit definition.

STM32G474RET6 + ELRS (CRSF) + 2x RS485 hub-motor links + Radxa host header,
5S battery input with ideal-diode reverse protection, monitored, a hot-swap
motor bus switch driven by a hardware safety chain (motor watchdog AND E-stop),
three independent 5 V rails and a 3V3 LDO.

Every part is listed once with its footprint and pin -> net map. gen_sch.py
turns this into the hierarchical KiCad schematic, gen_pcb.py into the board.

Net conventions
  * names in RAILS become power symbols (global)
  * "NC" places a no-connect flag
  * any other name is a signal; it becomes a hierarchical label automatically
    when used on more than one sheet, else a local label
Pins not listed get a no-connect flag and are reported by the generator.
"""

RAILS = ["GND", "+3V3", "+3V3A", "VBAT", "VMOT", "+5V_SBC", "+5V_ACT", "+5V_CTRL", "VBUS"]


class Part:
    def __init__(self, prefix, lib, value, fp, pins, desc="", dnp=False, rot=None, mpn=""):
        self.prefix, self.lib, self.value, self.fp, self.pins = prefix, lib, value, fp, pins
        self.desc, self.dnp, self.rot, self.mpn = desc, dnp, rot, mpn
        self.ref = None


class Group:
    def __init__(self, title, parts, note="", width=150):
        self.title, self.parts, self.note, self.width = title, parts, note, width


class Sheet:
    def __init__(self, key, title, color, groups, note="", columns=2):
        self.key, self.title, self.color, self.groups, self.note, self.columns = key, title, color, groups, note, columns


# ---------------------------------------------------------------- footprints
R0603 = "Resistor_SMD:R_0603_1608Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
C1206 = "Capacitor_SMD:C_1206_3216Metric"
CP10 = "Capacitor_SMD:CP_Elec_10x10"
R2512 = "Resistor_SMD:R_2512_6332Metric"
LED0603 = "LED_SMD:LED_0603_1608Metric"
SOD123 = "Diode_SMD:D_SOD-123"
SMC = "Diode_SMD:D_SMC"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
SOT235 = "Package_TO_SOT_SMD:SOT-23-5"
DPAK = "Package_TO_SOT_SMD:TO-252-2"
GH4 = "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal"
SH4 = "Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal"
PH3 = "Connector_JST:JST_PH_S3B-PH-K_1x03_P2.00mm_Horizontal"
H1x2 = "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical"
H1x3 = "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical"
H1x4 = "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical"
H1x5 = "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical"
TP = "TestPoint:TestPoint_Pad_D1.5mm"
XT30 = "Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal"
XT60 = "Connector_AMASS:AMASS_XT60PW-M_1x02_P7.20mm_Horizontal"


def R(value, a, b, fp=R0603, **kw):
    return Part("R", "Device:R", value, fp, {"1": a, "2": b}, **kw)


def C(value, a, b, fp=C0603, **kw):
    return Part("C", "Device:C", value, fp, {"1": a, "2": b}, **kw)


def CP(value, plus, minus, fp=CP10, **kw):
    return Part("C", "Device:C_Polarized", value, fp, {"1": plus, "2": minus}, **kw)


def LED(colour, anode, cathode):
    return Part("D", "Device:LED", colour, LED0603, {"2": anode, "1": cathode})


def TVS(value, k, a):
    return Part("D", "Device:D_TVS", value, SMC, {"1": k, "2": a})


def TESTPOINT(net):
    return Part("TP", "Connector:TestPoint", net, TP, {"1": net})


def GH(value, p1, p2, p3, p4):
    return Part("J", "Connector_Generic:Conn_01x04", value, GH4, {"1": p1, "2": p2, "3": p3, "4": p4})


# ================================================================ 1 POWER INPUT
power_input = Sheet("power_input", "Power input & protection", (230, 120, 60), columns=2, note=(
    "5S Li-ion 16.0-21.0 V. 30 A fuse, ideal-diode reverse protection (no diode drop), "
    "input TVS and bulk. INA228 measures pack current through the 1 mOhm shunt (Kelvin-route "
    "the sense lines) and bus voltage; VBAT_SENSE is a second, independent ADC divider."), groups=[
    Group("Battery input", [
        Part("J", "Connector_Generic:Conn_01x02", "XT60 battery", XT60, {"1": "VBAT_RAW", "2": "GND"}),
        Part("F", "Device:Fuse", "30A blade", "Fuse:Fuseholder_Blade_Mini_Keystone_3568", {"1": "VBAT_RAW", "2": "VBAT_FUSED"}),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "VBAT_RAW"}),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "GND"}),
    ], width=110),
    Group("Reverse-polarity ideal diode", [
        Part("U", "Power_Management:LM74700", "LM74700-Q1", "Package_TO_SOT_SMD:SOT-23-6",
             {"ANODE": "VBAT_FUSED", "CATHODE": "VBAT_PROT", "GATE": "RP_GATE", "VCAP": "RP_VCAP", "EN": "VBAT_FUSED", "GND": "GND"}),
        Part("Q", "Transistor_FET:Q_NMOS_GDS", "IPD80N04S4 40V", DPAK, {"1": "RP_GATE", "2": "VBAT_PROT", "3": "VBAT_FUSED"}),
        C("100n", "RP_VCAP", "VBAT_FUSED"),
        TVS("SMDJ26A", "VBAT_PROT", "GND"),
        CP("220u 35V", "VBAT_PROT", "GND"),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "VBAT_FUSED"}),
    ], width=150),
    Group("Pack current & voltage monitor", [
        R("1m 2512", "VBAT_PROT", "VBAT", fp=R2512),
        Part("U", "Sensor_Energy:INA228", "INA228", None,
             {"Vin+": "VBAT_PROT", "Vin-": "VBAT", "Vbus": "VBAT", "VS": "+3V3", "GND": "GND",
              "SDA": "I2C_SDA", "SCL": "I2C_SCL", "~{Alert}": "PWR_ALERT", "A0": "GND", "A1": "GND"}),
        C("100n", "+3V3", "GND"),
        R("100k", "VBAT", "VBAT_SENSE"),
        R("10k", "VBAT_SENSE", "GND"),
        C("100n", "VBAT_SENSE", "GND"),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "VBAT"}),
    ], width=150),
])

# ================================================================ 2 MOTOR POWER + SAFETY
motor_power = Sheet("motor_power", "Motor bus switch & safety chain", (215, 60, 70), columns=2, note=(
    "VMOT is switched by an LM5069 hot-swap (inrush and circuit-breaker limit). It is ON only while "
    "MOTOR_EN is high: WDG_OK (retriggerable one-shot fed by the control loop toggling MOTOR_WDG; "
    "~20 ms timeout) AND ESTOP_OK (normally-closed E-stop loop). A stalled MCU, reset, flashing, "
    "lost 3V3 or an open E-stop all remove motor power. Q4 only connects the UVLO divider while "
    "MOTOR_EN is high, so the default state is OFF."), groups=[
    Group("Hot-swap motor switch", [
        Part("U", "Power_Management:LM5069MM-1", "LM5069-1", None,
             {"VIN": "VBAT", "SENSE": "HS_SENSE", "GATE": "HS_GATE", "OUT": "VMOT", "UVLO": "HS_UVLO", "OVLO": "HS_OVLO",
              "TIMER": "HS_TIMER", "PWR": "HS_PWR", "PGD": "MOTOR_PG", "GND": "GND"}),
        R("2m 2512", "VBAT", "HS_SENSE", fp=R2512),
        Part("Q", "Transistor_FET:Q_NMOS_GDS", "IPD80N04S4 40V", DPAK, {"1": "HS_GATE", "2": "HS_SENSE", "3": "VMOT"}),
        C("47n", "HS_TIMER", "GND"),
        R("20k", "HS_PWR", "GND"),
        R("91k", "VBAT", "HS_OVLO"),
        R("10k", "HS_OVLO", "GND"),
        R("10k", "MOTOR_PG", "+3V3"),
        TVS("SMDJ26A", "VMOT", "GND"),
        CP("470u 35V", "VMOT", "GND"),
        CP("470u 35V", "VMOT", "GND"),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "VMOT"}),
    ], width=170),
    Group("UVLO enable (fail-safe OFF)", [
        Part("Q", "Transistor_FET:BSS138", "BSS138", SOT23, {"1": "MOTOR_EN", "2": "GND", "3": "EN_DRV"}),
        R("100k", "MOTOR_EN", "GND"),
        R("10k", "EN_DRV", "EN_PGATE"),
        R("10k", "VBAT", "EN_PGATE"),
        Part("Q", "Transistor_FET:BSS84", "BSS84", SOT23, {"1": "EN_PGATE", "2": "VBAT", "3": "UVLO_TOP"}),
        R("56k", "UVLO_TOP", "HS_UVLO"),
        R("10k", "HS_UVLO", "GND"),
    ], width=130),
    Group("Motor watchdog AND E-stop", [
        Part("U", "74xGxx:74LVC1G123", "74LVC1G123", "Package_SO:VSSOP-8_2.3x2mm_P0.5mm",
             {"~{A}": "GND", "B": "MOTOR_WDG", "~{CLR}": "+3V3", "Q": "WDG_OK", "Cext": "WDG_C", "RCext": "WDG_RC",
              "VCC": "+3V3", "GND": "GND"}),
        R("200k", "+3V3", "WDG_RC"),
        C("100n", "WDG_C", "WDG_RC"),
        C("100n", "+3V3", "GND"),
        Part("U", "74xGxx:74LVC1G08", "74LVC1G08", SOT235, {"1": "WDG_OK", "2": "ESTOP_OK", "4": "MOTOR_EN", "3": "GND", "5": "+3V3"}),
        C("100n", "+3V3", "GND"),
        Part("J", "Connector_Generic:Conn_01x02", "E-STOP (NC loop)", H1x2, {"1": "+3V3", "2": "ESTOP_OK"}),
        R("10k", "ESTOP_OK", "GND"),
        TESTPOINT("WDG_OK"), TESTPOINT("MOTOR_EN"), TESTPOINT("MOTOR_PG"),
    ], width=150),
    Group("Motor power outputs", [
        Part("J", "Connector_Generic:Conn_01x02", "XT30 motor L", XT30, {"1": "VMOT", "2": "GND"}),
        Part("J", "Connector_Generic:Conn_01x02", "XT30 motor R", XT30, {"1": "VMOT", "2": "GND"}),
    ], width=90),
])

# ================================================================ 3 REGULATORS
regulators = Sheet("regulators", "Regulators", (225, 175, 40), columns=2, note=(
    "Three independent 5 V rails so servos, LEDs or the Radxa can never brown out the controller. "
    "Placeholder footprints are Pololu D36V-series step-down modules (pin order VIN, GND, VOUT, EN, PG); "
    "replace with on-board bucks later. The MCU 3V3 comes from its own LDO on +5V_CTRL."), groups=[
    Group("5 V host: Radxa, lidar, camera, ToF (5 A)", [
        Part("U", "Connector_Generic:Conn_01x05", "Pololu D36V50F5", H1x5,
             {"1": "VBAT", "2": "GND", "3": "+5V_SBC", "4": "NC", "5": "NC"}),
        C("47u", "+5V_SBC", "GND", fp=C1206),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "+5V_SBC"}),
    ], width=110),
    Group("5 V actuators: servos, LED ring, torch (3 A)", [
        Part("U", "Connector_Generic:Conn_01x05", "Pololu D36V28F5", H1x5,
             {"1": "VBAT", "2": "GND", "3": "+5V_ACT", "4": "NC", "5": "NC"}),
        C("47u", "+5V_ACT", "GND", fp=C1206),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "+5V_ACT"}),
    ], width=110),
    Group("5 V control: ELRS, RS485, logic (0.6 A)", [
        Part("U", "Connector_Generic:Conn_01x04", "Pololu D36V6F5", H1x4, {"1": "VBAT", "2": "GND", "3": "+5V_CTRL", "4": "NC"}),
        C("22u", "+5V_CTRL", "GND", fp=C1206),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "+5V_CTRL"}),
    ], width=110),
    Group("3V3 MCU LDO", [
        Part("U", "Regulator_Linear:TLV75733PDBV", "TLV75733", None, {"IN": "+5V_CTRL", "EN": "+5V_CTRL", "OUT": "+3V3", "GND": "GND"}),
        C("1u", "+5V_CTRL", "GND"),
        C("1u", "+3V3", "GND"),
        R("1k", "+3V3", "PWR_LED_A"),
        LED("green 3V3", "PWR_LED_A", "GND"),
    ], width=120),
])

# ================================================================ 4 MCU
mcu_pins = {
    "VBAT": "+3V3", "VDD": "+3V3", "VSS": "GND", "VSSA": "GND", "VDDA": "+3V3A", "VREF+": "+3V3A",
    "PC13": "USER_BTN", "PC14": "NC", "PC15": "NC", "PF0": "HSE_IN", "PF1": "HSE_OUT", "PG10": "NRST",
    "PC0": "ADC_5V_SBC", "PC1": "ADC_5V_ACT", "PC2": "ADC_VMOT", "PC3": "ADC_TEMP",
    "PA0": "VBAT_SENSE", "PA1": "RS485L_DE", "PA2": "RS485L_TX", "PA3": "RS485L_RX",
    "PA4": "IMU_CS", "PA5": "IMU_SCK", "PA6": "IMU_MISO", "PA7": "IMU_MOSI",
    "PC4": "MOTOR_WDG", "PC5": "PWR_ALERT", "PB0": "IMU_INT1", "PB1": "IMU_INT2", "PB2": "ESTOP_OK",
    "PB10": "RS485R_TX", "PB11": "RS485R_RX", "PB12": "CAN_RX", "PB13": "CAN_TX", "PB14": "RS485R_DE", "PB15": "MOTOR_PG",
    "PC6": "SERVO1", "PC7": "SERVO2", "PC8": "SERVO3", "PC9": "SERVO4", "PA8": "LED_RING_3V3",
    "PA9": "HOST_TX", "PA10": "HOST_RX", "PA11": "USB_DM", "PA12": "USB_DP", "PA13": "SWDIO", "PA14": "SWCLK",
    "PA15": "I2C_SCL", "PB9": "I2C_SDA", "PC10": "ELRS_TX", "PC11": "ELRS_RX", "PC12": "AUX_TX", "PD2": "AUX_RX",
    "PB3": "SWO", "PB4": "BUZZER", "PB5": "HOST_GPIO", "PB6": "TORCH_PWM", "PB7": "STATUS_LED", "PB8": "BOOT0",
}
mcu = Sheet("mcu", "STM32G474 controller", (60, 120, 210), columns=2, note=(
    "STM32G474RET6, 170 MHz Cortex-M4F. USART2/USART3 drive the RS485 links with hardware DE, "
    "UART4 is the ELRS CRSF input, USART1 (PA9/PA10) is the Radxa link and also the ROM bootloader "
    "port; the Radxa controls BOOT0 and NRST. MOTOR_WDG must be a software-toggled GPIO, never a timer "
    "output. Verify the pin map in STM32CubeMX before layout."), groups=[
    Group("Microcontroller", [
        Part("U", "MCU_ST_STM32G4:STM32G474RETx", "STM32G474RET6", None, mcu_pins),
    ], width=150),
    Group("Decoupling", [
        C("100n", "+3V3", "GND"), C("100n", "+3V3", "GND"), C("100n", "+3V3", "GND"), C("100n", "+3V3", "GND"),
        C("100n", "+3V3", "GND"), C("4u7", "+3V3", "GND", fp=C0805),
        Part("FB", "Device:FerriteBead", "600R@100MHz", "Inductor_SMD:L_0603_1608Metric", {"1": "+3V3", "2": "+3V3A"}),
        C("1u", "+3V3A", "GND"), C("100n", "+3V3A", "GND"),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "+3V3A"}),
    ], width=120),
    Group("24 MHz clock", [
        Part("Y", "Device:Crystal_GND24", "24MHz 10pF", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm", {"1": "HSE_IN", "3": "HSE_OUT", "2": "GND", "4": "GND"}),
        C("10p", "HSE_IN", "GND"), C("10p", "HSE_OUT", "GND"),
    ], width=90),
    Group("Reset, boot, user button", [
        R("10k", "BOOT0", "GND"),
        C("100n", "NRST", "GND"),
        Part("SW", "Switch:SW_Push", "RESET", "Button_Switch_SMD:SW_SPST_TL3342", {"1": "NRST", "2": "GND"}),
        Part("SW", "Switch:SW_Push", "USER", "Button_Switch_SMD:SW_SPST_TL3342", {"1": "USER_BTN", "2": "GND"}),
        R("10k", "+3V3", "USER_BTN"),
    ], width=110),
    Group("Debug (SWD) and USB-C", [
        Part("J", "Connector:Conn_ARM_JTAG_SWD_10", "SWD", "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD",
             {"1": "+3V3", "2": "SWDIO", "3": "GND", "5": "GND", "4": "SWCLK", "6": "SWO", "8": "NC", "9": "GND", "10": "NRST"}),
        Part("J", "Connector:USB_C_Receptacle_USB2.0_16P", "USB-C (DFU, console)", "Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal",
             {"VBUS": "VBUS", "GND": "GND", "CC1": "USB_CC1", "CC2": "USB_CC2", "D+": "USB_DP", "D-": "USB_DM", "SHIELD": "GND"}),
        Part("U", "Power_Protection:USBLC6-2SC6", "USBLC6-2SC6", None, {"I/O1": "USB_DM", "I/O2": "USB_DP", "VBUS": "VBUS", "GND": "GND"}),
        R("5k1", "USB_CC1", "GND"), R("5k1", "USB_CC2", "GND"),
        Part("D", "Device:D_Schottky", "B5819W", SOD123, {"2": "VBUS", "1": "+5V_CTRL"}),
        Part("FLG", "power:PWR_FLAG", "PWR_FLAG", "", {"1": "VBUS"}),
    ], width=150),
    Group("Status LED", [
        R("1k", "STATUS_LED", "STAT_LED_A"),
        LED("blue status", "STAT_LED_A", "GND"),
    ], width=80),
])

# ================================================================ 5 RS485


def rs485(side, label):
    s = side
    return Group("Hub motor %s (motor ID %s)" % (label, "1" if s == "L" else "2"), [
        Part("U", "Interface_UART:THVD1450D", "SN65HVD3082EP", "Package_DIP:DIP-8_W7.62mm_Socket",
             {"RO": "RS485%s_RX" % s, "~{RE}": "RS485%s_DE" % s, "DE": "RS485%s_DE" % s, "DI": "RS485%s_TX" % s,
              "GND": "GND", "A": "RS485%s_A" % s, "B": "RS485%s_B" % s, "VCC": "+5V_CTRL"},
             desc="any standard 8-pin half-duplex RS485 transceiver with idle-bus fail-safe, socketed"),
        C("100n", "+5V_CTRL", "GND"),
        R("120R", "RS485%s_A" % s, "TERM%s" % s),
        Part("JP", "Jumper:Jumper_2_Open", "TERM", H1x2, {"1": "TERM%s" % s, "2": "RS485%s_B" % s}),
        R("680R", "RS485%s_A" % s, "+5V_CTRL", dnp=True),
        R("680R", "RS485%s_B" % s, "GND", dnp=True),
        Part("D", "Diode:SM712_SOT23", "SM712", None, {"A1": "RS485%s_A" % s, "A2": "RS485%s_B" % s, "common": "GND"}),
        GH("Motor %s RS485" % label, "RS485%s_A" % s, "RS485%s_B" % s, "GND", "GND"),
    ], width=150)


rs485_sheet = Sheet("rs485", "Hub motor RS485 links", (40, 160, 120), columns=2, note=(
    "One transceiver per motor (115200 baud, 10-byte frames every 2.5 ms). Socketed DIP-8 for easy swap. "
    "Fail-safe receivers read an idle or shorted bus as logic 1, which stops corrupt frames. Fit the "
    "120 Ohm termination jumper at the board end; bias resistors are DNP footprints. Twist A/B with GND."),
    groups=[rs485("L", "LEFT"), rs485("R", "RIGHT")])

# ================================================================ 6 COMMS
comms = Sheet("comms", "Radio, host link & CAN", (140, 90, 200), columns=2, note=(
    "ELRS receiver on UART4 (CRSF 420 kbaud) replaces the Bluetooth gamepad. The Radxa header carries "
    "+5V_SBC to the host, the host UART (same line protocol as today), BOOT0/NRST control for "
    "stm32flash, and passes the lidar UART and ToF I2C through to their connectors. VERIFY the Cubie "
    "A7A 40-pin mapping before ordering."), groups=[
    Group("ELRS receiver (CRSF)", [
        GH("ELRS RX", "+5V_CTRL", "GND", "ELRS_RX", "ELRS_TX"),
    ], width=90),
    Group("Radxa Cubie A7A header (2x20)", [
        Part("J", "Connector_Generic:Conn_02x20_Odd_Even", "Radxa 40-pin", "Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical",
             {"1": "HOST_3V3", "2": "+5V_SBC", "3": "TOF_SDA", "4": "+5V_SBC", "5": "TOF_SCL", "6": "GND",
              "8": "HOST_RX", "9": "GND", "10": "HOST_TX", "11": "NRST_CTL", "13": "BOOT0_CTL", "14": "GND",
              "15": "HOST_GPIO", "16": "LIDAR_TX", "18": "LIDAR_RX", "20": "GND", "25": "GND", "30": "GND",
              "34": "GND", "39": "GND"}),
        R("1k", "NRST_CTL", "NRST"),
        R("1k", "BOOT0_CTL", "BOOT0"),
    ], width=150),
    Group("Lidar and ToF pass-through", [
        GH("RPLIDAR C1", "+5V_SBC", "GND", "LIDAR_TX", "LIDAR_RX"),
        Part("J", "Connector_Generic:Conn_01x04", "VL53L5CX (Qwiic)", SH4, {"1": "GND", "2": "HOST_3V3", "3": "TOF_SDA", "4": "TOF_SCL"}),
    ], width=110),
    Group("CAN (optional, FDCAN2)", [
        Part("U", "Interface_CAN_LIN:SN65HVD230", "SN65HVD230", None,
             {"D": "CAN_TX", "R": "CAN_RX", "VCC": "+3V3", "GND": "GND", "Rs": "GND", "Vref": "NC", "CANH": "CANH", "CANL": "CANL"}),
        C("100n", "+3V3", "GND"),
        R("120R", "CANH", "CAN_TERM"),
        Part("JP", "Jumper:Jumper_2_Open", "TERM", H1x2, {"1": "CAN_TERM", "2": "CANL"}),
        GH("CAN", "CANH", "CANL", "GND", "GND"),
    ], width=150),
    Group("Spare UART", [
        GH("AUX UART", "+3V3", "AUX_TX", "AUX_RX", "GND"),
    ], width=90),
])

# ================================================================ 7 IO
io = Sheet("io", "Sensors & I/O", (60, 170, 190), columns=2, note=(
    "IMU on SPI1 with data-ready interrupts; mount it near the axle line, away from bucks and motor "
    "current paths (footprint uses the pin-compatible ISM330DHCX symbol, verify ISM6HG256X pinout). "
    "The WS2812 ring needs a 5 V data level: 74AHCT1G125 buffer. Servos, ring and torch run from +5V_ACT."),
    groups=[
    Group("IMU", [
        Part("U", "Sensor_Motion:ISM330DHCX", "ISM6HG256X", None,
             {"VDDIO": "+3V3", "VDD": "+3V3", "GND": "GND", "CS": "IMU_CS", "SCL": "IMU_SCK", "SDA": "IMU_MOSI",
              "SDO/SA0": "IMU_MISO", "INT1": "IMU_INT1", "INT2": "IMU_INT2", "SDx": "GND", "SCx": "GND"}),
        C("100n", "+3V3", "GND"), C("100n", "+3V3", "GND"),
    ], width=130),
    Group("I2C expansion (Qwiic)", [
        Part("J", "Connector_Generic:Conn_01x04", "Qwiic 1", SH4, {"1": "GND", "2": "+3V3", "3": "I2C_SDA", "4": "I2C_SCL"}),
        Part("J", "Connector_Generic:Conn_01x04", "Qwiic 2", SH4, {"1": "GND", "2": "+3V3", "3": "I2C_SDA", "4": "I2C_SCL"}),
        R("4k7", "+3V3", "I2C_SDA"), R("4k7", "+3V3", "I2C_SCL"),
    ], width=120),
    Group("LED ring (WS2812, 5 V data)", [
        Part("U", "74xGxx:74AHCT1G125", "74AHCT1G125", SOT235, {"1": "GND", "2": "LED_RING_3V3", "4": "LED_RING_5V", "3": "GND", "5": "+5V_ACT"}),
        C("100n", "+5V_ACT", "GND"),
        R("330R", "LED_RING_5V", "LED_DATA"),
        CP("100u", "+5V_ACT", "GND", fp="Capacitor_SMD:CP_Elec_6.3x7.7"),
        Part("J", "Connector_Generic:Conn_01x03", "LED ring", PH3, {"1": "+5V_ACT", "2": "LED_DATA", "3": "GND"}),
    ], width=130),
    Group("Servos (pan, zoom, 2 spare)", [
        Part("J", "Connector_Generic:Conn_01x03", "Servo pan", H1x3, {"1": "SERVO1", "2": "+5V_ACT", "3": "GND"}),
        Part("J", "Connector_Generic:Conn_01x03", "Servo zoom", H1x3, {"1": "SERVO2", "2": "+5V_ACT", "3": "GND"}),
        Part("J", "Connector_Generic:Conn_01x03", "Servo 3", H1x3, {"1": "SERVO3", "2": "+5V_ACT", "3": "GND"}),
        Part("J", "Connector_Generic:Conn_01x03", "Servo 4", H1x3, {"1": "SERVO4", "2": "+5V_ACT", "3": "GND"}),
    ], width=120),
    Group("Torch and buzzer", [
        Part("Q", "Transistor_FET:AO3400A", "AO3400A", None, {"G": "TORCH_PWM", "S": "GND", "D": "TORCH_N"}),
        R("100k", "TORCH_PWM", "GND"),
        Part("J", "Connector_Generic:Conn_01x02", "Torch LED", H1x2, {"1": "+5V_ACT", "2": "TORCH_N"}),
        Part("Q", "Transistor_BJT:MMBT3904", "MMBT3904", None, {"B": "BUZ_B", "E": "GND", "C": "BUZ_N"}),
        R("1k", "BUZZER", "BUZ_B"),
        Part("BZ", "Device:Buzzer", "buzzer 5V", "Buzzer_Beeper:Buzzer_12x9.5RM7.6", {"1": "+5V_CTRL", "2": "BUZ_N"}),
        Part("D", "Device:D_Schottky", "B5819W", SOD123, {"1": "+5V_CTRL", "2": "BUZ_N"}),
    ], width=150),
    Group("Rail and temperature monitors (ADC)", [
        R("30k", "+5V_SBC", "ADC_5V_SBC"), R("10k", "ADC_5V_SBC", "GND"),
        R("30k", "+5V_ACT", "ADC_5V_ACT"), R("10k", "ADC_5V_ACT", "GND"),
        R("100k", "VMOT", "ADC_VMOT"), R("10k", "ADC_VMOT", "GND"),
        R("10k", "+3V3", "ADC_TEMP"),
        Part("TH", "Device:Thermistor_NTC", "10k NTC", R0603, {"1": "ADC_TEMP", "2": "GND"}),
    ], width=150),
])

SHEETS = [power_input, motor_power, regulators, mcu, rs485_sheet, comms, io]

# Root-sheet parts (mechanical)
ROOT_PARTS = [Part("H", "Mechanical:MountingHole", "M3", "MountingHole:MountingHole_3.2mm_M3", {}) for _ in range(4)]

PROJECT = {"name": "baselink", "title": "2PyBot BaseLink controller", "rev": "0.1", "company": "2PyBot",
           "date": "2026-11-24"}
