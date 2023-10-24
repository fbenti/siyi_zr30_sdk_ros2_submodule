"""
Python implementation of ZR10 SDK by SIYI
ZR10 webpage: http://en.siyi.biz/en/Gimbal%20Camera/ZR10/overview/
Author : Mohamed Abdelkader
Email: mohamedashraf123@gmail.com
Copyright 2022

"""
from os import stat
from crc16_python import crc16_str_swap
import logging
from utils import toHex


class FirmwareMsg:
    seq = 0
    code_board_ver = ""
    gimbal_firmware_ver = ""
    zoom_firmware_ver = ""


class HardwareIDMsg:
    seq = 0
    id = ""


class AutoFocusMsg:
    seq = 0
    success = False


class ManualZoomMsg:
    seq = 0
    level = -1


class CurrentZoomValueMsg:
    seq = 0
    zoom_int = -1
    zoom_float = -1.0


class AbsoluteZoomMsg:
    seq = 0
    success = -1


class ManualFocusMsg:
    seq = 0
    success = False


class GimbalSpeedMsg:
    seq = 0
    success = False


class CenterMsg:
    seq = 0
    success = False


class RecordingMsg:
    seq = 0
    state = -1
    OFF = 0
    ON = 1
    TF_EMPTY = 2
    TD_DATA_LOSS = 3


class MountDirMsg:
    seq = 0
    dir = -1
    NORMAL = 0
    UPSIDE = 1


class MotionModeMsg:
    seq = 0
    mode = -1
    LOCK = 0
    FOLLOW = 1
    FPV = 2


class FuncFeedbackInfoMsg:
    seq = 0
    info_type = None
    SUCCESSFUL = 0
    PHOTO_FAIL = 1
    HDR_ON = 2
    HDR_OFF = 3
    RECROD_FAIL = 4


class AttitdueMsg:
    seq = 0
    stamp = 0  # seconds
    yaw = 0.0
    pitch = 0.0
    roll = 0.0
    yaw_speed = 0.0  # deg/s
    pitch_speed = 0.0
    roll_speed = 0.0


class MaxZoomValueMsg:
    seq = 0
    zoom_max_int = 0
    zoom_max_float = 0.0


class SendControlAngleToGimbalMsg:
    seq = 0
    yaw = 0.0
    pitch = 0.0
    current_yaw = 0.0
    current_pitch = 0.0
    current_roll = 0.0


class COMMAND:
    ABSOLUTE_ZOOM_AUTO_FOCUS = "0f"
    AUTO_FOCUS = "04"
    CENTER = "08"
    FUNCTION_FEEDBACK_INFO = "0b"
    GIMBAL_ROTATION_SPEED = "07"
    MANUAL_FOCUS = "06"
    MANUAL_ZOOM_AUTO_FOCUS = "05"
    PHOTO_AND_RECORD = "0c"
    REQUEST_GIMBAL_ATTITUDE = "0d"
    REQUEST_GIMBAL_CAMERA_FIRMWARE = "01"
    REQUEST_GIMBAL_CAMERA_HARDWARE_ID = "02"
    REQUEST_GIMBAL_CAMERA_PRESENT_WORKING_MODE = "19"
    REQUEST_GIMBAL_CONFIGURATION_INFO = "0a"
    REQUEST_MAX_ZOOM_VALUE_IN_PRESENT = "16"
    REQUEST_ZOOM_VALUE_IN_PRESENT = "18"
    SEND_CONTROL_ANGLE_TO_GIMBAL = "0e"


#############################################
class SIYIMESSAGE:
    """
    Structure of SIYI camera messages
    """

    def __init__(self, debug=False) -> None:
        self._debug = debug  # print debug messages
        if self._debug:
            d_level = logging.DEBUG
        else:
            d_level = logging.INFO
        LOG_FORMAT = (
            "[%(levelname)s] %(asctime)s [SIYIMessage::%(funcName)s] :\t%(message)s"
        )
        logging.basicConfig(format=LOG_FORMAT, level=d_level)
        self._logger = logging.getLogger(self.__class__.__name__)

        self.HEADER = "5566"  # STX, 2 bytes
        self._ctr = "01"

        self._seq = 0

        self._cmd_id = "00"  # 1 byte

        self._data_len = 0

        # String of data byes (in hex)
        self._data = ""

        self._crc16 = "0000"  # low byte (2 characters) on the left!

    def incrementSEQ(self, val):
        """
        Increments sequence number by one, converts them to hex, and revereses the byte order.

        Params
        --
        - val [int] Integer value , max is 65535

        Returns
        --
        seq_str: [string] String value of the sequence number in reveresed byte order
        """

        if not isinstance(val, int):
            self._logger.warning("Sequence value is not integer. Returning zero")
            return "0000"
        if val > 65535:
            self._logger.warning(
                "Sequence value is greater than 65535. Resetting to zero"
            )
            self._seq = 0
            return "0000"
        if val < 0:
            self._logger.warning("Sequence value is negative. Resetting to zero")
            return "0000"

        seq = val + 1
        self._seq = seq

        seq_hex = hex(seq)
        seq_hex = seq_hex[2:]  # remove '0x'
        if len(seq_hex) == 3:
            seq_hex = "0" + seq_hex
        elif len(seq_hex) == 1:
            seq_hex = "000" + seq_hex
        elif len(seq_hex) == 2:
            seq_str = "00" + seq_hex
        else:
            seq = "0000"

        low_b = seq_hex[-2:]
        high_b = seq_hex[0:2]
        seq_str = low_b + high_b

        return seq_str

    def computeDataLen(self, data):
        """
        Computes the data lenght (number of bytes) of data, and return a string of two bytes in reveresed order

        Params
        --
        data [string] string of data bytes in hex

        Returns
        --
        [string] String of two bytes (for characters), in reversed order, represents length of data in hex
        """

        if not isinstance(data, str):
            self._logger.error("Data is not of type string")
            return "0000"
        # We expect number of chartacters to be even (each byte is represented by two cahrs e.g. '0A')
        if (len(data) % 2) != 0:
            data = "0" + data  # Pad 0 from the left, as sometimes it's ignored!
        L = int(len(data) / 2)

        len_hex = hex(L)
        len_hex = len_hex[2:]  # remove '0x'
        if len(len_hex) == 3:
            len_hex = "0" + len_hex
        elif len(len_hex) == 1:
            len_hex = "000" + len_hex
        elif len(len_hex) == 2:
            len_hex = "00" + len_hex
        else:
            len_hex = "0000"

        low_b = len_hex[-2:]
        high_b = len_hex[0:2]
        len_str = low_b + high_b

        return len_str

    def decodeMsg(self, msg):
        """
        Decodes messages string, and returns the DATA bytes.

        Params
        --
        msg: [str] full message stinf in hex

        Returns
        --
        - data [str] string of hexadecimal of data bytes.
        - data_len [int] Number of data bytes
        - cmd_id [str] command ID
        - seq [int] message sequence
        """
        data = None
        if not isinstance(msg, str):
            self._logger.error("Input message is not a string")
            return data

        # 10 bytes: STX+CTRL+Data_len+SEQ+CMD_ID+CRC16
        #            2 + 1  +    2   + 2 +   1  + 2
        MINIMUM_DATA_LENGTH = 10 * 2
        if len(msg) < MINIMUM_DATA_LENGTH:
            self._logger.error("No data to decode")
            return data

        # Now we got minimum amount of data. Check if we have enough
        # Data length, bytes are reversed, according to SIYI SDK
        low_b = msg[6:8]  # low byte
        high_b = msg[8:10]  # high byte
        data_len = high_b + low_b
        data_len = int("0x" + data_len, base=16)
        char_len = (
            data_len * 2
        )  # number of characters. Each byte is represented by two characters in hex, e.g. '0A'= 2 chars

        # check crc16, if msg is OK!
        msg_crc = msg[-4:]  # last 4 characters
        payload = msg[:-4]
        expected_crc = crc16_str_swap(payload)
        if expected_crc != msg_crc:
            self._logger.error(
                "CRC16 is not valid. Got %s. Expected %s. Message might be corrupted!",
                msg_crc,
                expected_crc,
            )
            return data

        # Sequence
        low_b = msg[10:12]  # low byte
        high_b = msg[12:14]  # high byte
        seq_hex = high_b + low_b
        seq = int("0x" + seq_hex, base=16)

        # CMD ID
        cmd_id = msg[14:16]

        # DATA
        if data_len > 0:
            data = msg[16 : 16 + char_len]
        else:
            data = ""

        self._data = data
        self._data_len = data_len
        self._cmd_id = cmd_id

        return data, data_len, cmd_id, seq

    def encodeMsg(self, data, cmd_id):
        """
        Encodes a msg according to SDK protocol

        Returns
        --
        [str] Encoded msg. Empty string if crc16 is not successful
        """
        seq = self.incrementSEQ(self._seq)
        data_len = self.computeDataLen(data)
        msg_front = self.HEADER+self._ctr+data_len+seq+cmd_id+data
        msg_front = self.HEADER + self._ctr + data_len + "0000" + cmd_id + data
        crc = crc16_str_swap(msg_front)
        # if cmd_id == COMMAND.SEND_CONTROL_ANGLE_TO_GIMBAL:
        #     print("data: ", data)
        #     print("cmd_id: ", cmd_id)
        #     print("crc: ", crc)
        if crc is not None:
            msg = msg_front + crc
            self._logger.debug("Encoded msg: %s", msg)
            return msg
        else:
            self._logger.error("Could not encode message. crc16 is None")
            return ""

    ########################################################
    #               Message definitions                    #
    ########################################################
    def absoluteZoomMsg(self, zoom_level: int, frac_zoom_level: int):
        """
        Use Absolute zoom
        """
        if zoom_level > 30:
            zoom_level = 30
        if zoom_level < 1:
            zoom_level = 1

        if frac_zoom_level > 9:
            zoom_level = 9
        if frac_zoom_level < 0:
            zoom_level = 0

        data1 = toHex(zoom_level, 8)
        data2 = toHex(frac_zoom_level, 8)
        data = data1 + data2
        cmd_id = COMMAND.ABSOLUTE_ZOOM_AUTO_FOCUS
        return self.encodeMsg(data, cmd_id)

    def autoFocusMsg(self):
        """
        Auto focus msg
        """
        data = "01"
        cmd_id = COMMAND.AUTO_FOCUS
        return self.encodeMsg(data, cmd_id)

    def gimbalAttitudeMsg(self):
        """
        Acquire Gimbal Attiude msg
        """
        data = ""
        cmd_id = COMMAND.REQUEST_GIMBAL_ATTITUDE
        return self.encodeMsg(data, cmd_id)

    def firmwareVersionMsg(self):
        """
        Returns message string of the Acqsuire Firmware Version msg
        """
        data = ""
        cmd_id = COMMAND.REQUEST_GIMBAL_CAMERA_FIRMWARE
        return self.encodeMsg(data, cmd_id)

    def hardwareIdMsg(self):
        """
        Returns message string for the Acquire Hardware ID
        """
        data = ""
        cmd_id = COMMAND.REQUEST_GIMBAL_CAMERA_HARDWARE_ID
        return self.encodeMsg(data, cmd_id)

    def gimbalCameraWorkingModeMsg(self):
        """
        Returns message string for Request Gimbal Camera's Present Working Mode
        """
        data = ""
        cmd_id = COMMAND.REQUEST_GIMBAL_CAMERA_PRESENT_WORKING_MODE
        return self.encodeMsg(data, cmd_id)

    def centerGimbalMsg(self):
        """
        Center gimbal msg
        """
        data = "01"
        cmd_id = COMMAND.CENTER
        return self.encodeMsg(data, cmd_id)

    def closeFocusMsg(self):
        """
        Focus -1 Msg
        """
        data = "ff"
        cmd_id = COMMAND.MANUAL_FOCUS
        return self.encodeMsg(data, cmd_id)

    def currentZoomValueMsg(self):
        """
        Current Zoom Value.
        """
        data = ""
        cmd_id = COMMAND.REQUEST_ZOOM_VALUE_IN_PRESENT
        return self.encodeMsg(data, cmd_id)

    def gimbalConfigurationInfo(self):
        """
        Gimbal status information msg
        """
        data = ""
        cmd_id = COMMAND.REQUEST_GIMBAL_CONFIGURATION_INFO
        return self.encodeMsg(data, cmd_id)

    def gimbalSpeedMsg(self, yaw_speed, pitch_speed):
        """
        Gimbal rotation Msg.
        Values -100~0~100: Negative and positive represent two directions,
        higher or lower the number is away from 0, faster the rotation speed is.
        Send 0 when released from control command and gimbal stops rotation.

        Params
        --
        - yaw_speed [int] in degrees
        - pitch_speed [int] in degrees
        """
        if yaw_speed > 100:
            yaw_speed = 100
        if yaw_speed < -100:
            yaw_speed = -100

        if pitch_speed > 100:
            pitch_speed = 100
        if pitch_speed < -100:
            pitch_speed = -100

        data1 = toHex(yaw_speed, 8)
        data2 = toHex(pitch_speed, 8)
        data = data1 + data2
        cmd_id = COMMAND.GIMBAL_ROTATION_SPEED
        return self.encodeMsg(data, cmd_id)

    def stopFocusMsg(self):
        """
        Focus 0 Msg
        """
        data = "00"
        cmd_id = COMMAND.MANUAL_FOCUS
        return self.encodeMsg(data, cmd_id)

    def followModeMsg(self):
        """
        Follow mode msg
        """
        data = "04"
        cmd_id = COMMAND.PHOTO_AND_RECORD
        return self.encodeMsg(data, cmd_id)

    def fpvModeMsg(self):
        """
        FPV mode msg
        """
        data = "05"
        cmd_id = COMMAND.PHOTO_AND_RECORD
        return self.encodeMsg(data, cmd_id)

    def functionFeedbackMsg(self):
        """
        Function feedback information msg
        """
        data = ""
        cmd_id = COMMAND.FUNCTION_FEEDBACK_INFO
        return self.encodeMsg(data, cmd_id)

    def lockModeMsg(self):
        """
        Lock mode msg
        """
        data = "03"
        cmd_id = COMMAND.PHOTO_AND_RECORD
        return self.encodeMsg(data, cmd_id)

    def longFocusMsg(self):
        """
        Focus 1 Msg
        """
        data = "01"
        cmd_id = COMMAND.MANUAL_FOCUS
        return self.encodeMsg(data, cmd_id)

    def maxZoomValueMsg(self):
        """
        MaxZoomvalue
        """
        data = ""
        cmd_id = COMMAND.REQUEST_MAX_ZOOM_VALUE_IN_PRESENT
        return self.encodeMsg(data, cmd_id)

    def startStopRecordingMsg(self):
        """
        Video Record msg
        """
        data = "02"
        cmd_id = COMMAND.PHOTO_AND_RECORD
        return self.encodeMsg(data, cmd_id)

    def takePhotoMsg(self):
        """
        Take photo msg
        """
        data = "00"
        cmd_id = COMMAND.PHOTO_AND_RECORD
        return self.encodeMsg(data, cmd_id)

    def stopZoomMsg(self):
        """
        Stop Zoom Msg
        """
        data = "00"
        cmd_id = COMMAND.MANUAL_ZOOM_AUTO_FOCUS

        return self.encodeMsg(data, cmd_id)

    def zoomInMsg(self):
        """
        Zoom in Msg
        """
        data = "01"
        cmd_id = COMMAND.MANUAL_ZOOM_AUTO_FOCUS

        return self.encodeMsg(data, cmd_id)

    def zoomOutMsg(self):
        """
        Zoom out Msg
        """
        data = "ff"
        cmd_id = COMMAND.MANUAL_ZOOM_AUTO_FOCUS

        return self.encodeMsg(data, cmd_id)

    def sendControlAngleToGimbalMsg(self, yaw:int, pitch:int):
        """
        Control gimbal
        """
        data1 = toHex(yaw, 16)
        data2 = toHex(pitch, 16)
        print(data1)
        print(data2)
        data = data1 + data2
        cmd_id = COMMAND.SEND_CONTROL_ANGLE_TO_GIMBAL
        print(self.encodeMsg(data, cmd_id))
        # return "556601040000000e0000ffa63b11"
        return self.encodeMsg(data, cmd_id)

