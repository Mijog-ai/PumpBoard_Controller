"""
Parameter Definitions for STM32F407 PumpBoard Controller
Maps to HengLiPumpCmd.h command indices
"""

class ParameterDefinitions:
    """Parameter indices matching g_HLCmdMap[] array in STM32 firmware"""

    # Read-only Information Parameters (0x00-0x1F)
    INF_SOFTWARE_VERSION = 0x00
    INF_HARDWARE_VERSION = 0x01
    INF_ADC_ANG_FDB = 0x02
    INF_ADC_PRS_FDB = 0x03
    INF_ADC_CUR_A_FDB = 0x04
    INF_ADC_CUR_B_FDB = 0x05
    INF_PWM_A = 0x06
    INF_PWM_B = 0x07
    INF_ANG_FDB = 0x08
    INF_PRS_FDB = 0x09
    INF_CUR_A_FDB = 0x0A
    INF_CUR_B_FDB = 0x0B
    INF_ANG_FDB_D = 0x0C
    INF_PRS_FDB_D = 0x0D

    # Enable Commands (0x20-0x2F)
    ENA_PUMP_START = 0x20
    ENA_ANG_LEAK = 0x21
    ENA_PRS_LOOP = 0x22
    ENA_PWR_LOOP = 0x23
    ENA_PARA_SAVE = 0x24
    ENA_SIMULATION_MODE = 0x25

    # Control Commands (0x30-0x3F)
    CTRL_TILT_ANG_REF = 0x30
    CTRL_PRESSURE_REF = 0x31
    CTRL_CUR_A_REF = 0x32
    CTRL_CUR_B_REF = 0x33
    CTRL_TORQUE_LIMIT = 0x34

    # Current Loop PID Parameters (0x40-0x4F)
    PI_CUR_A_KP = 0x40
    PI_CUR_A_KI = 0x41
    PI_CUR_A_KD = 0x42
    PI_CUR_A_OUTPUT_MAX = 0x43
    PI_CUR_A_OUTPUT_MIN = 0x44
    PI_CUR_A_SUMMAX = 0x45

    PI_CUR_B_KP = 0x46
    PI_CUR_B_KI = 0x47
    PI_CUR_B_KD = 0x48
    PI_CUR_B_OUTPUT_MAX = 0x49
    PI_CUR_B_OUTPUT_MIN = 0x4A
    PI_CUR_B_SUMMAX = 0x4B

    # Angle Loop PID Parameters (0x50-0x5F)
    PI_ANG_KP = 0x50
    PI_ANG_KI = 0x51
    PI_ANG_KD = 0x52
    PI_ANG_KV = 0x53
    PI_ANG_OUTPUT_MAX = 0x54
    PI_ANG_OUTPUT_MIN = 0x55
    PI_ANG_SUMMAX = 0x56

    # Pressure Loop PID Parameters (0x60-0x6F)
    PI_PRS_KP = 0x60
    PI_PRS_KI = 0x61
    PI_PRS_KD = 0x62
    PI_PRS_KV = 0x63
    PI_PRS_OUTPUT_MAX = 0x64
    PI_PRS_OUTPUT_MIN = 0x65
    PI_PRS_SUMMAX = 0x66

    # Filter Time Constants (0x70-0x7F)
    FILTER_ANG_REF = 0x70
    FILTER_ANG_FDB = 0x71
    FILTER_PRS_FDB = 0x72
    FILTER_PWR_FDB = 0x73

    # Calibration Parameters (0x80-0x9F)
    CAL_ANG_MIN_ADC = 0x80
    CAL_ANG_MID_ADC = 0x81
    CAL_ANG_MAX_ADC = 0x82
    CAL_ANG_PERUNIT_SCOPE = 0x83

    CAL_PRS_MIN_ADC = 0x84
    CAL_PRS_MID_ADC = 0x85
    CAL_PRS_MAX_ADC = 0x86
    CAL_PRS_MAX_SCOPE = 0x87

    # Swing/Dither Parameters (0xA0-0xAF)
    SWING_FREQ = 0xA0
    SWING_AMP = 0xA1

    # Loop Cycle Periods (0xB0-0xBF)
    CYCLE_ANG_PI = 0xB0
    CYCLE_PRS_PI = 0xB1
    CYCLE_CUR_PI = 0xB2
    CYCLE_ANG_D_FILTER = 0xB3
    CYCLE_PRS_D_FILTER = 0xB4


class ParameterInfo:
    """Parameter metadata for GUI display and validation"""

    PARAMETERS = {
        # Real-time Feedback (Read-only)
        'angle_feedback': {
            'index': ParameterDefinitions.INF_ANG_FDB,
            'name': 'Angle Feedback',
            'unit': '% (0-100)',
            'scale': 0.01,  # Per-unit to percentage
            'min': 0,
            'max': 10000,
            'readonly': True,
            'category': 'Feedback'
        },
        'pressure_feedback': {
            'index': ParameterDefinitions.INF_PRS_FDB,
            'name': 'Pressure Feedback',
            'unit': 'bar',
            'scale': 0.01,
            'min': 0,
            'max': 600,
            'readonly': True,
            'category': 'Feedback'
        },
        'current_a_feedback': {
            'index': ParameterDefinitions.INF_CUR_A_FDB,
            'name': 'Current A Feedback',
            'unit': 'mA',
            'scale': 1.0,
            'min': 0,
            'max': 3000,
            'readonly': True,
            'category': 'Feedback'
        },
        'current_b_feedback': {
            'index': ParameterDefinitions.INF_CUR_B_FDB,
            'name': 'Current B Feedback',
            'unit': 'mA',
            'scale': 1.0,
            'min': 0,
            'max': 3000,
            'readonly': True,
            'category': 'Feedback'
        },
        'pwm_a': {
            'index': ParameterDefinitions.INF_PWM_A,
            'name': 'PWM A Output',
            'unit': '%',
            'scale': 0.01,
            'min': 0,
            'max': 21000,
            'readonly': True,
            'category': 'Feedback'
        },
        'pwm_b': {
            'index': ParameterDefinitions.INF_PWM_B,
            'name': 'PWM B Output',
            'unit': '%',
            'scale': 0.01,
            'min': 0,
            'max': 21000,
            'readonly': True,
            'category': 'Feedback'
        },

        # Control Setpoints
        'angle_reference': {
            'index': ParameterDefinitions.CTRL_TILT_ANG_REF,
            'name': 'Angle Reference',
            'unit': '% (0-100)',
            'scale': 0.01,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'Control'
        },
        'pressure_reference': {
            'index': ParameterDefinitions.CTRL_PRESSURE_REF,
            'name': 'Pressure Reference',
            'unit': 'bar',
            'scale': 0.01,
            'min': 0,
            'max': 600,
            'readonly': False,
            'category': 'Control'
        },
        'current_a_reference': {
            'index': ParameterDefinitions.CTRL_CUR_A_REF,
            'name': 'Current A Reference',
            'unit': 'mA',
            'scale': 1.0,
            'min': 0,
            'max': 3000,
            'readonly': False,
            'category': 'Control'
        },
        'current_b_reference': {
            'index': ParameterDefinitions.CTRL_CUR_B_REF,
            'name': 'Current B Reference',
            'unit': 'mA',
            'scale': 1.0,
            'min': 0,
            'max': 3000,
            'readonly': False,
            'category': 'Control'
        },
        'torque_limit': {
            'index': ParameterDefinitions.CTRL_TORQUE_LIMIT,
            'name': 'Torque Limit',
            'unit': 'Nm',
            'scale': 1.0,
            'min': 0,
            'max': 250,
            'readonly': False,
            'category': 'Control'
        },

        # Enable Flags
        'enable_pump_start': {
            'index': ParameterDefinitions.ENA_PUMP_START,
            'name': 'Enable Pump Start',
            'unit': 'bool',
            'scale': 1.0,
            'min': 0,
            'max': 1,
            'readonly': False,
            'category': 'Enable'
        },
        'enable_pressure_loop': {
            'index': ParameterDefinitions.ENA_PRS_LOOP,
            'name': 'Enable Pressure Loop',
            'unit': 'bool',
            'scale': 1.0,
            'min': 0,
            'max': 1,
            'readonly': False,
            'category': 'Enable'
        },
        'enable_angle_leak': {
            'index': ParameterDefinitions.ENA_ANG_LEAK,
            'name': 'Enable Angle Leakage',
            'unit': 'bool',
            'scale': 1.0,
            'min': 0,
            'max': 1,
            'readonly': False,
            'category': 'Enable'
        },
        'enable_power_loop': {
            'index': ParameterDefinitions.ENA_PWR_LOOP,
            'name': 'Enable Power Loop',
            'unit': 'bool',
            'scale': 1.0,
            'min': 0,
            'max': 1,
            'readonly': False,
            'category': 'Enable'
        },

        # Angle Loop PID
        'angle_kp': {
            'index': ParameterDefinitions.PI_ANG_KP,
            'name': 'Angle Kp',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Angle'
        },
        'angle_ki': {
            'index': ParameterDefinitions.PI_ANG_KI,
            'name': 'Angle Ki',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Angle'
        },
        'angle_kd': {
            'index': ParameterDefinitions.PI_ANG_KD,
            'name': 'Angle Kd',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Angle'
        },
        'angle_kv': {
            'index': ParameterDefinitions.PI_ANG_KV,
            'name': 'Angle Kv',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Angle'
        },

        # Pressure Loop PID
        'pressure_kp': {
            'index': ParameterDefinitions.PI_PRS_KP,
            'name': 'Pressure Kp',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Pressure'
        },
        'pressure_ki': {
            'index': ParameterDefinitions.PI_PRS_KI,
            'name': 'Pressure Ki',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Pressure'
        },
        'pressure_kd': {
            'index': ParameterDefinitions.PI_PRS_KD,
            'name': 'Pressure Kd',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Pressure'
        },
        'pressure_kv': {
            'index': ParameterDefinitions.PI_PRS_KV,
            'name': 'Pressure Kv',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Pressure'
        },

        # Current Loop A PID
        'current_a_kp': {
            'index': ParameterDefinitions.PI_CUR_A_KP,
            'name': 'Current A Kp',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Current_A'
        },
        'current_a_ki': {
            'index': ParameterDefinitions.PI_CUR_A_KI,
            'name': 'Current A Ki',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 30000,
            'readonly': False,
            'category': 'PID_Current_A'
        },
        'current_a_kd': {
            'index': ParameterDefinitions.PI_CUR_A_KD,
            'name': 'Current A Kd',
            'unit': '',
            'scale': 1.0,
            'min': 0,
            'max': 10000,
            'readonly': False,
            'category': 'PID_Current_A'
        },
    }

    @staticmethod
    def get_parameter_by_index(index):
        """Get parameter info by command index"""
        for key, param in ParameterInfo.PARAMETERS.items():
            if param['index'] == index:
                return key, param
        return None, None

    @staticmethod
    def get_parameters_by_category(category):
        """Get all parameters in a category"""
        return {k: v for k, v in ParameterInfo.PARAMETERS.items()
                if v['category'] == category}
