#!/usr/bin/env python
""" For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

import time, serial

class DWM1001_API_COMMANDS:
        DOUBLE_ENTER    = b'\r\r'   # ASCII char for double Enter
        SINGLE_ENTER    = b'\r'     # ASCII char for single Enter
        HELP            = b'?'      # Display help
        QUIT            = b'quit'   # Quit API shell mode
        GC              = b'gc'     # Clears GPIO pin
        GG              = b'gg'     # Reads GPIO pin level
        GS              = b'gs'     # Sets GPIO as output and sets its value
        GT              = b'gt'     # Toggle GPIO(must be an output)
        F               = b'f'      # Show free memory on the heap
        PS              = b'ps'     # Show info about running threads
        PMS             = b'pms'    # Show power managements tasks. IDL means that task is idle. USE means that task is allocated in the power management
        RESET           = b'reset'  # reset the dev board
        UT              = b'ut'     # Show device uptime
        FRST            = b'frst'   # Factory reset
        TWI             = b'twi'    # General purpose I2C/TWI read
        AID             = b'aid'    # Read ACC device ID
        AV              = b'av'     # Rad ACC values
        LES             = b'les'    # Show distances to ranging anchors and the position if location engine is enabled
        LEC             = b'lec'    # Show measurement and position in CSV format
        LEP             = b'lep'    # Show position in CSV format.Sending this command multiple times will turn on/off this functionality.
        SI              = b'si'     # System Info
        NMG             = b'nmg'    # Get node mode info
        NMO             = b'nmo'    # Enable passive offline option and resets the node
        NMP             = b'nmp'    # Enable active offline option and resets the node.
        NMA             = b'nma'    # Configures node to as anchor, active and reset the node.
        NMI             = b'nmi'    # Configures node to as anchor initiator, active and reset the node.
        NMT             = b'nmt'    # Configures node to as tag, active and reset the node
        NMTL            = b'nmtl'   # Configures node to as tag, active, low power and resets the node.
        BPC             = b'bpc'    # Toggle UWB bandwidth / tx power compensation.
        LA              = b'la'     # Show anchor list
        STG             = b'stg'    # Display statistics
        STC             = b'stc'    # Clears statistics
        TLV             = b'tlv'    # Parses given tlv frame, see section 4 for valid TLV commands
        AURS            = b'aurs'   # Set position update rate. See section 4.3.3 for more detail.
        AURG            = b'aurg'   # Get position update rate. See section 4.3.4 for more details
        APG             = b'apg'    # Get position of the node.See section 3.4.2 for more detail
        APS             = b'aps'    # Set position of the node.See section 3.4.2for more detail
        ACAS            = b'acas'   # Configures node as anchor with given options
        ACTS            = b'acts'   # Configures node as tag with given options
        NIS             = b'nis'    # Set Network ID  

class DWM1001_UART_API:
        def initSerial(self, serial_port):
                """
                Initialize port and dwm1001 api
                Parameters
                ----------
                Returns
                ----------
                """

                # Serial port settings
                self.serialPortDWM1001 = serial.Serial(
                port = serial_port,
                baudrate = 115200,
                parity = serial.PARITY_ODD,
                stopbits = serial.STOPBITS_TWO,
                bytesize = serial.SEVENBITS,
                timeout = 0.1
                )

                # close the serial port in case the previous run didn't closed it properly
                self.serialPortDWM1001.close()
                # sleep for one sec
                time.sleep(1)
                # open serial port
                self.serialPortDWM1001.open()

                # check if the serial port is opened
                if(self.serialPortDWM1001.isOpen()):
                        print("[INFO] Port opened: "+ str(self.serialPortDWM1001.name) )
                        # start sending commands to the board so we can initialize the board
                        self.initializeDWM1001API()
                        # give some time to DWM1001 to wake up
                        time.sleep(2)
                else:
                        print("[INFO] Can't open port: "+ str(self.serialPortDWM1001.name))

        def initializeDWM1001API(self):
                """ Initialize dwm10801 api, by sending sending bytes
                Parameters
                ----------
                Returns
                ----------
                """
                # reset incase previuos run didn't close properly
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
                # send ENTER two times in order to access api
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                # sleep for half a second
                time.sleep(0.5)
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                # sleep for half second
                time.sleep(0.5)
                # send a third one - just in case
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                time.sleep(0.5)

        def handleKeyboardInterrupt(self):
                """ Handles keyboard interruption
                Parameters
                ----------
                Returns
                ----------
                """
                print("[INFO] Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        def quit(self):
                """ Quit and send reset command to dev board
                Parameters
                ----------
                Returns
                ----------
                """
                print("[INFO] Quitting, and sending reset command to dev board")
                # self.serialPortDWM1001.reset_input_buffer()
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                time.sleep(1.0)
                serialReadLine = self.serialPortDWM1001.read_until().decode()
                if "reset" in serialReadLine:
                        print("[INFO] succesfully closed ")
                        self.serialPortDWM1001.close()

        def readSerial(self, command, verbose = False):
                """ Read serial string and return data as array
                Parameters
                ----------
                command : b
                        DWM1001_API_COMMANDS
                Returns
                -------
                array_data : array
                        data as array
                """
                try:
                        serial_read_line = self.serialPortDWM1001.read_until().decode()
                except:
                        return ['']

                array_data = [x.strip() for x in serial_read_line.strip().split(' ')]
                if verbose: print(serial_read_line)
                if command in array_data: return ['']
                return array_data

        def getDataFromSerial(self, dwm_request, verbose = False, read_attempts = 10):
                """ Tries to read retrieved 'data'
                from serial port sending 'command'
                Parameters
                ----------
                command : b (Bytes)
                        DWM1001_API_COMMANDS command
                read_attempts : int
                        attempts to read 'data'
                Returns
                -------
                data : array
                        retrieved data
                """

                # Read data
                is_data_valid = False
                n_attempts = 0
                while is_data_valid == False:
                        data = self.readSerial(dwm_request, verbose)
                        is_data_valid = dwm_request.validness(data)
                        n_attempts += 1
                        if n_attempts > read_attempts: # max attempts to read serial
                                return []
                return data

        def les(self):
                # set anchor position lecture 
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LES)
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                time.sleep(0.5)

        def acas(self, args):
                """Configure module as an anchor with a given configuration
                Parameters
                ----------
                args: list of integers
                        list of configuration arguments in the following order
                        [initiator, bridge_en, enc_en, leds, ble, uwb, fw_upd]
                """
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.ACAS)
                time.sleep(0.05)

                for arg in args:
                        self.serialPortDWM1001.write(bytes(' ' + str(arg)))
                        time.sleep(0.05)

                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                time.sleep(0.5)

        def acts(self, args):
                """Configure module a tag with a given configuration
                Parameters
                ----------
                args: list of integers
                        list of configuration arguments in the following order
                        [meas_mode, stnry_en, low_pwr, loc_en, enc, leds, ble, uwb, fw_upd]
                """
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.ACTS)
                time.sleep(0.05)

                for arg in args:
                        self.serialPortDWM1001.write(bytes(' ' + str(arg)))
                        time.sleep(0.05)

                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                time.sleep(0.5)

        def aurs(self, args):
                """ Set position update rate
                Parameters
                ----------
                args: list of integers
                        list of configuration arguments in the following order
                        [ur, urs]
                        see p42 DWM1001 Firmware API Guide for reference
                """
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.AURS)
                time.sleep(0.05)

                for arg in args:
                        self.serialPortDWM1001.write(bytes(' ' + format(arg,'02x').upper()))
                        time.sleep(0.05)

                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                time.sleep(0.5)

        def nis(self, network_id):
                """ Set position update rate
                Parameters
                ----------
                args: string formated as '0x0000'
                        network id    
                """
                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.NIS)
                time.sleep(0.05)

                self.serialPortDWM1001.write(bytes(' 0x'))
                time.sleep(0.05)
                self.serialPortDWM1001.write(bytes(network_id[2:4]))
                time.sleep(0.05)
                self.serialPortDWM1001.write(bytes(network_id[4:6]))
                time.sleep(0.05)

                self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
                time.sleep(0.5)

class DWMRangingReq(object):
        def __init__(self, is_location_engine_enabled = False):
                self.command = DWM1001_API_COMMANDS.LES
                self.is_location_engine_enabled = is_location_engine_enabled

        def validness(self, data):
                """
                Checks if all elements in anchor 'data' have at least 18 bytes
                and if location engine is enabled tag pose at least 21 bytes                
                :param: array of strings
                :returns: bool
                """
                if self.is_location_engine_enabled:
                        data, tag_pose_est = [data[:-2], data[-1]]
                        if len(tag_pose_est) < 18:
                                return False
                for element in data:
                        if len(element) < 21:
                                return False
                return True

class DWMAccReq(object):
        def __init__(self):
                self.command = DWM1001_API_COMMANDS.AV

        def validness(self, data):
                """
                Checks if 'data' have at least 24 bytes
                :param: string
                :returns: bool
                """
                if 'acc:' not in data or 'x' not in data or \
                        'y' not in data or 'z' not in data:
                        return False
                return True