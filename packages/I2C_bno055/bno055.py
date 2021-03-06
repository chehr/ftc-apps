#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#
import os, struct, sys
from datetime import datetime
from time import *
from TouchStyle import *
import smbus, xlwt

# Logging settings
ACCEL_DATA          =   False
MAG_DATA            =   True
GYRO_DATA           =   True
EULER_DATA          =   True
QUATERNION_DATA     =   True
LINEAR_ACCEL_DATA   =   False
GRAVITY_DATA        =   True
TEMP_DATA           =   True
VISUAL_VIEW         =   True
DATARATE            = 100       # Data sampling rate [ms]
LOGGING_TIME        = 300        # Logging time [s]


excel_xf = xlwt.easyxf
data=[]
t0 = time()
t1 = t0
appdir = os.path.dirname(os.path.realpath(__file__))


class ValueWidget(QSlider):
    def __init__(self, parent=None):
        QSlider.__init__(self, Qt.Horizontal, parent)
        self.setDisabled(True)
        self.setRange(-100, 100)

class TinyLabel(QLabel):
    def __init__(self, str, parent=None):
        super(TinyLabel, self).__init__(str, parent)
        self.setObjectName("tinylabel")

class TouchGuiApplication(TouchApplication):
    def __init__(self, args):
        TouchApplication.__init__(self, args)

        translator = QTranslator()
        path = os.path.dirname(os.path.realpath(__file__))
        translator.load(QLocale.system(), os.path.join(path, "bno055_"))
        self.installTranslator(translator)

        # create the empty main window
        self.w = TouchWindow("BNO055")
       
        self.vbox = QVBoxLayout()
        self.vbox.addStretch()
        
        #Start I2C Communication
        try:
            self.bno = BNO055()
            self.bno.begin()
        except IOError:
            self.bno = None

            lbl = QLabel("Unable to connect to " +
                         "BNO055 sensor on EXT port.")
            lbl.setObjectName("smalllabel")
            lbl.setWordWrap(True)
            lbl.setAlignment(Qt.AlignCenter)
            self.vbox.addWidget(lbl)

        if self.bno:
            # Temperature
            self.temp_lbl = TinyLabel("Temperature:", self.w)
            self.temp_lbl.setAlignment(Qt.AlignCenter)
            self.vbox.addWidget(self.temp_lbl)
                 
            # Accelerometer
            lbl = TinyLabel("Accelerometer [mg]", self.w)
            lbl.setAlignment(Qt.AlignCenter)
            self.vbox.addWidget(lbl)

            # Acc XYZ value
            self.axyz_lbl = TinyLabel("Acceleration:", self.w)
            #self.axyz_lbl.setAlignment(Qt.AlignCenter)
            self.vbox.addWidget(self.axyz_lbl)

            self.acc_grid_w = QWidget()
            self.acc_grid = QGridLayout()
            self.acc_grid.setVerticalSpacing(0)
            self.acc_grid_w.setLayout(self.acc_grid)
            self.acc_x_value = ValueWidget(self.acc_grid_w)
            self.acc_grid.addWidget(TinyLabel("X:", self.acc_grid_w),0,0)
            self.acc_grid.addWidget(self.acc_x_value,0,1)
            self.acc_y_value = ValueWidget(self.w)
            self.acc_grid.addWidget(TinyLabel("Y:", self.acc_grid_w),1,0)
            self.acc_grid.addWidget(self.acc_y_value,1,1)
            self.acc_z_value = ValueWidget(self.w)
            self.acc_grid.addWidget(TinyLabel("Z:", self.acc_grid_w),2,0)
            self.acc_grid.addWidget(self.acc_z_value,2,1)
            self.vbox.addWidget(self.acc_grid_w)

            # Magnetometer
            lbl = TinyLabel("Magnetometer [nT]", self.w)
            lbl.setAlignment(Qt.AlignCenter)
            self.vbox.addWidget(lbl)

            # Mag XYZ value
            self.mxyz_lbl = TinyLabel("Magnetometer:", self.w)
            #self.mxyz_lbl.setAlignment(Qt.AlignCenter)
            self.vbox.addWidget(self.mxyz_lbl)

            self.mag_grid_w = QWidget()
            self.mag_grid = QGridLayout()
            self.mag_grid.setVerticalSpacing(0)
            self.mag_grid_w.setLayout(self.mag_grid)
            self.mag_x_value = ValueWidget(self.mag_grid_w)
            self.mag_grid.addWidget(TinyLabel("X:", self.mag_grid_w),0,0)
            self.mag_grid.addWidget(self.mag_x_value,0,1)
            self.mag_y_value = ValueWidget(self.w)
            self.mag_grid.addWidget(TinyLabel("Y:", self.mag_grid_w),1,0)
            self.mag_grid.addWidget(self.mag_y_value,1,1)
            self.mag_z_value = ValueWidget(self.w)
            self.mag_grid.addWidget(TinyLabel("Z:", self.mag_grid_w),2,0)
            self.mag_grid.addWidget(self.mag_z_value,2,1)
            self.vbox.addWidget(self.mag_grid_w)

            #Todo: Auswahl hinzu ob Daten visual angezeigt werden oder gespeichert, um die Performance zu verbessern.

            # Print BNO055 software revision and other diagnostic data.
            accel_rev, mag_rev, gyro_rev, sw_rev, bl_rev = self.bno.getRevInfo()
            print('Software version:   {0}'.format(sw_rev))
            print('Bootloader version: {0}'.format(bl_rev))
            print('Accelerometer ID:   0x{0:02X}'.format(accel_rev))
            print('Magnetometer ID:    0x{0:02X}'.format(mag_rev))
            print('Gyroscope ID:       0x{0:02X}\n'.format(gyro_rev))
  
            # start a qtimer to read the sensor data
            self.timer = QTimer(self)
            #self.timer.setSingleShot(False)
            self.timer.timeout.connect(lambda: self.__on_timer())   # lambda will avoid the evaluation of the fuction call
            self.timer.start(DATARATE)                              # Sampling rate

            print("\nstart: %s" % ctime(t0))
           
        self.vbox.addStretch()
        self.w.centralWidget.setLayout(self.vbox)

        self.w.show()
        self.exec_()
      
    def __on_timer(self):
        # get the accelerometer and magnetometer values
        if ACCEL_DATA:
            a = self.bno.getVector(BNO055.VECTOR_ACCELEROMETER)     # Tuple
            self.acc_x_value.setValue(a[0])
            self.acc_y_value.setValue(a[1])
            self.acc_z_value.setValue(a[2])
            axyz=str(a)
            self.axyz_lbl.setText("xyz: {0:20}".format(axyz))
        if MAG_DATA:   
            m = self.bno.getVector(BNO055.VECTOR_MAGNETOMETER)      # Tuple
            self.mag_x_value.setValue(m[0])
            self.mag_y_value.setValue(m[1])
            self.mag_z_value.setValue(m[2])
            mxyz=str(m)
            self.mxyz_lbl.setText("xyz: {0:20}".format(mxyz))
        if GYRO_DATA:
            g = self.bno.getVector(BNO055.VECTOR_GYROSCOPE)         # Tuple
        if EULER_DATA:
            e = self.bno.getVector(BNO055.VECTOR_EULER)             # Tuple
        if QUATERNION_DATA:
            q=self.bno.getQuat()
        if LINEAR_ACCEL_DATA:
            l = self.bno.getVector(BNO055.VECTOR_LINEARACCEL)       # Tuple
        if GRAVITY_DATA:
            ag = self.bno.getVector(BNO055.VECTOR_GRAVITY)          # Tuple
        if TEMP_DATA:
            t=self.bno.getTemp()
            self.temp_lbl.setText("Temperature: {0:d}°C".format(t))

        # Read Sensor data for SD-Card storage as Excel file
        global t0, t1, data
        dataline=[datetime.now(),time()-t1]
        if ACCEL_DATA:
            dataline.extend(list(a))
        if MAG_DATA:
            dataline.extend(list(m))
        if GYRO_DATA:
            dataline.extend(list(g))
        if EULER_DATA:
            dataline.extend(list(e))
        if QUATERNION_DATA:
            dataline.extend(list(q))
        if LINEAR_ACCEL_DATA:
            dataline.extend(list(l))
        if GRAVITY_DATA:
            dataline.extend(list(ag))
        if TEMP_DATA:
            dataline.append(t)
        data.extend([dataline])
        
        t1 = time()
        dt = t1-t0
        if dt >= LOGGING_TIME:                                         # Data logging time
            self.timer.stop()
            print("since starting elapsed %.2f s" % (dt))
            first_line = ['Time [s]', "dt [s]"]
            if ACCEL_DATA:
                first_line.extend(['AX [mg]', 'AY [mg]', 'AZ [mg]'])
            if MAG_DATA:
                first_line.extend(['MX [nT]', 'MY [nT]', 'MZ [nT]'])
            if GYRO_DATA:
                first_line.extend(['GX [deg/s]', 'GY [deg/s]', 'GZ [deg/s]'])
            if EULER_DATA:
                first_line.extend(['Heading [0deg..360deg]', 'Roll [-90deg..+90deg]', 'Pitch [-180deg..+180deg]'])
            if QUATERNION_DATA:
                first_line.extend(['QW [-]', 'QX [-]', 'QY [-]', 'QZ [-]'])              
            if LINEAR_ACCEL_DATA:
                first_line.extend(['LAX [mg]', 'LAY [mg]', 'LAZ [mg]'])
            if GRAVITY_DATA:
                first_line.extend(['GAX [mg]', 'GAY [mg]', 'GAZ [mg]'])
            if TEMP_DATA:
                first_line.append('Temperature [deg C]')               

            kinds =  'date int int int int int int int int int int int int int int int int int int int int int int int int'.split()
            first_line_xf = excel_xf('font: bold on; align: wrap on, vert centre, horiz center')
            kind_to_xf_map = {
                        'date': excel_xf(num_format_str='hh:mm:ss'), #DD.MM.YYYY removed
                        'int': excel_xf(num_format_str='0.000'),
                        'text': excel_xf(),
                        }
            data_xfs = [kind_to_xf_map[k] for k in kinds] 
            print("Daten auf SD Karte schreiben...")
            #self.__write_xls('home/ftc/apps/a9fb0a90-29e9-11e7-9598-0800200c9a66/Datalogger.xls', 'BNO055', first_line, data, first_line_xf, data_xfs)
            self.__write_xls(appdir+'/Datalogger.xls', 'BNO055', first_line, data, first_line_xf, data_xfs)
                        
    # Write data to Excel file
    def __write_xls(self, file_name, sheet_name, headings, data, heading_xf, data_xfs):
        book = xlwt.Workbook()
        sheet = book.add_sheet(sheet_name)
        rowx = 0
        for colx, value in enumerate(headings):
            sheet.write(rowx, colx, value, heading_xf)
            sheet.set_panes_frozen(True) # frozen headings instead of split panes
            sheet.set_horz_split_pos(rowx+1) # in general, freeze after last heading row
            sheet.set_remove_splits(True) # if user does unfreeze, don't leave a split there
        for row in data:
            rowx += 1
            for colx, value in enumerate(row):
                sheet.write(rowx, colx, value, data_xfs[colx])
            book.save(file_name)

        
class BNO055:
   BNO055_ADDRESS_A                 = 0x28
   BNO055_ADDRESS_B                 = 0x29
   BNO055_ID                        = 0xA0

   # Power mode settings
   POWER_MODE_NORMAL                = 0X00
   POWER_MODE_LOWPOWER              = 0X01
   POWER_MODE_SUSPEND               = 0X02

   # Operation mode settings
   OPERATION_MODE_CONFIG            = 0X00
   OPERATION_MODE_ACCONLY           = 0X01
   OPERATION_MODE_MAGONLY           = 0X02
   OPERATION_MODE_GYRONLY           = 0X03
   OPERATION_MODE_ACCMAG            = 0X04
   OPERATION_MODE_ACCGYRO           = 0X05
   OPERATION_MODE_MAGGYRO           = 0X06
   OPERATION_MODE_AMG               = 0X07
   OPERATION_MODE_IMUPLUS           = 0X08
   OPERATION_MODE_COMPASS           = 0X09
   OPERATION_MODE_M4G               = 0X0A
   OPERATION_MODE_NDOF_FMC_OFF      = 0X0B
   OPERATION_MODE_NDOF              = 0X0C #Baseline

   # Output vector type
   VECTOR_ACCELEROMETER             = 0x08
   VECTOR_MAGNETOMETER              = 0x0E
   VECTOR_GYROSCOPE                 = 0x14
   VECTOR_EULER                     = 0x1A
   VECTOR_LINEARACCEL               = 0x28
   VECTOR_GRAVITY                   = 0x2E

   # REGISTER DEFINITION START
   BNO055_PAGE_ID_ADDR              = 0X07

   BNO055_CHIP_ID_ADDR              = 0x00
   BNO055_ACCEL_REV_ID_ADDR         = 0x01
   BNO055_MAG_REV_ID_ADDR           = 0x02
   BNO055_GYRO_REV_ID_ADDR          = 0x03
   BNO055_SW_REV_ID_LSB_ADDR        = 0x04
   BNO055_SW_REV_ID_MSB_ADDR        = 0x05
   BNO055_BL_REV_ID_ADDR            = 0X06

   # Accel data register
   BNO055_ACCEL_DATA_X_LSB_ADDR        = 0X08
   BNO055_ACCEL_DATA_X_MSB_ADDR        = 0X09
   BNO055_ACCEL_DATA_Y_LSB_ADDR        = 0X0A
   BNO055_ACCEL_DATA_Y_MSB_ADDR        = 0X0B
   BNO055_ACCEL_DATA_Z_LSB_ADDR        = 0X0C
   BNO055_ACCEL_DATA_Z_MSB_ADDR        = 0X0D

   # Mag data register
   BNO055_MAG_DATA_X_LSB_ADDR          = 0X0E
   BNO055_MAG_DATA_X_MSB_ADDR          = 0X0F
   BNO055_MAG_DATA_Y_LSB_ADDR          = 0X10
   BNO055_MAG_DATA_Y_MSB_ADDR          = 0X11
   BNO055_MAG_DATA_Z_LSB_ADDR          = 0X12
   BNO055_MAG_DATA_Z_MSB_ADDR          = 0X13

   # Gyro data registers
   BNO055_GYRO_DATA_X_LSB_ADDR         = 0X14
   BNO055_GYRO_DATA_X_MSB_ADDR         = 0X15
   BNO055_GYRO_DATA_Y_LSB_ADDR         = 0X16
   BNO055_GYRO_DATA_Y_MSB_ADDR         = 0X17
   BNO055_GYRO_DATA_Z_LSB_ADDR         = 0X18
   BNO055_GYRO_DATA_Z_MSB_ADDR         = 0X19

   # Euler data registers
   BNO055_EULER_H_LSB_ADDR              = 0X1A
   BNO055_EULER_H_MSB_ADDR              = 0X1B
   BNO055_EULER_R_LSB_ADDR              = 0X1C
   BNO055_EULER_R_MSB_ADDR              = 0X1D
   BNO055_EULER_P_LSB_ADDR              = 0X1E
   BNO055_EULER_P_MSB_ADDR              = 0X1F

   # Quaternion data registers
   BNO055_QUATERNION_DATA_W_LSB_ADDR      = 0X20
   BNO055_QUATERNION_DATA_W_MSB_ADDR      = 0X21
   BNO055_QUATERNION_DATA_X_LSB_ADDR      = 0X22
   BNO055_QUATERNION_DATA_X_MSB_ADDR      = 0X23
   BNO055_QUATERNION_DATA_Y_LSB_ADDR      = 0X24
   BNO055_QUATERNION_DATA_Y_MSB_ADDR      = 0X25
   BNO055_QUATERNION_DATA_Z_LSB_ADDR      = 0X26
   BNO055_QUATERNION_DATA_Z_MSB_ADDR      = 0X27

   # Linear acceleration data registers
   BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR    = 0X28
   BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR    = 0X29
   BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR    = 0X2A
   BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR    = 0X2B
   BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR    = 0X2C
   BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR    = 0X2D

   # Gravity data registers
   BNO055_GRAVITY_DATA_X_LSB_ADDR         = 0X2E
   BNO055_GRAVITY_DATA_X_MSB_ADDR         = 0X2F
   BNO055_GRAVITY_DATA_Y_LSB_ADDR         = 0X30
   BNO055_GRAVITY_DATA_Y_MSB_ADDR         = 0X31
   BNO055_GRAVITY_DATA_Z_LSB_ADDR         = 0X32
   BNO055_GRAVITY_DATA_Z_MSB_ADDR         = 0X33

   # Temperature data register
   BNO055_TEMP_ADDR                 = 0X34 #1 LSB = 1°C or 2F

   # Status registers
   BNO055_CALIB_STAT_ADDR           = 0X35
   BNO055_SELFTEST_RESULT_ADDR      = 0X36
   BNO055_INTR_STAT_ADDR            = 0X37

   BNO055_SYS_CLK_STAT_ADDR         = 0X38
   BNO055_SYS_STAT_ADDR             = 0X39
   BNO055_SYS_ERR_ADDR              = 0X3A

   # Unit selection register
   BNO055_UNIT_SEL_ADDR             = 0X3B
   BNO055_DATA_SELECT_ADDR          = 0X3C

   # Mode registers
   BNO055_OPR_MODE_ADDR             = 0X3D
   BNO055_PWR_MODE_ADDR             = 0X3E

   BNO055_SYS_TRIGGER_ADDR          = 0X3F
   BNO055_TEMP_SOURCE_ADDR          = 0X40 #Select Temperature source from Accelerometer (0x0) or Gyroscope (0x1)

   # Axis remap registers
   BNO055_AXIS_MAP_CONFIG_ADDR      = 0X41
   BNO055_AXIS_MAP_SIGN_ADDR        = 0X42

   # SIC registers
   BNO055_SIC_MATRIX_0_LSB_ADDR     = 0X43
   BNO055_SIC_MATRIX_0_MSB_ADDR     = 0X44
   BNO055_SIC_MATRIX_1_LSB_ADDR     = 0X45
   BNO055_SIC_MATRIX_1_MSB_ADDR     = 0X46
   BNO055_SIC_MATRIX_2_LSB_ADDR     = 0X47
   BNO055_SIC_MATRIX_2_MSB_ADDR     = 0X48
   BNO055_SIC_MATRIX_3_LSB_ADDR     = 0X49
   BNO055_SIC_MATRIX_3_MSB_ADDR     = 0X4A
   BNO055_SIC_MATRIX_4_LSB_ADDR     = 0X4B
   BNO055_SIC_MATRIX_4_MSB_ADDR     = 0X4C
   BNO055_SIC_MATRIX_5_LSB_ADDR     = 0X4D
   BNO055_SIC_MATRIX_5_MSB_ADDR     = 0X4E
   BNO055_SIC_MATRIX_6_LSB_ADDR     = 0X4F
   BNO055_SIC_MATRIX_6_MSB_ADDR     = 0X50
   BNO055_SIC_MATRIX_7_LSB_ADDR     = 0X51
   BNO055_SIC_MATRIX_7_MSB_ADDR     = 0X52
   BNO055_SIC_MATRIX_8_LSB_ADDR     = 0X53
   BNO055_SIC_MATRIX_8_MSB_ADDR     = 0X54

   # Accelerometer Offset registers
   ACCEL_OFFSET_X_LSB_ADDR          = 0X55
   ACCEL_OFFSET_X_MSB_ADDR          = 0X56
   ACCEL_OFFSET_Y_LSB_ADDR          = 0X57
   ACCEL_OFFSET_Y_MSB_ADDR          = 0X58
   ACCEL_OFFSET_Z_LSB_ADDR          = 0X59
   ACCEL_OFFSET_Z_MSB_ADDR          = 0X5A

   # Magnetometer Offset registers
   MAG_OFFSET_X_LSB_ADDR            = 0X5B
   MAG_OFFSET_X_MSB_ADDR            = 0X5C
   MAG_OFFSET_Y_LSB_ADDR            = 0X5D
   MAG_OFFSET_Y_MSB_ADDR            = 0X5E
   MAG_OFFSET_Z_LSB_ADDR            = 0X5F
   MAG_OFFSET_Z_MSB_ADDR            = 0X60

   # Gyroscope Offset registers
   GYRO_OFFSET_X_LSB_ADDR           = 0X61
   GYRO_OFFSET_X_MSB_ADDR           = 0X62
   GYRO_OFFSET_Y_LSB_ADDR           = 0X63
   GYRO_OFFSET_Y_MSB_ADDR           = 0X64
   GYRO_OFFSET_Z_LSB_ADDR           = 0X65
   GYRO_OFFSET_Z_MSB_ADDR           = 0X66

   # Radius registers
   ACCEL_RADIUS_LSB_ADDR            = 0X67
   ACCEL_RADIUS_MSB_ADDR            = 0X68
   MAG_RADIUS_LSB_ADDR              = 0X69
   MAG_RADIUS_MSB_ADDR              = 0X6A

   # REGISTER DEFINITION END


   def __init__(self, sensorId=-1, address=0x28):
      self._sensorId = sensorId
      self._address = address
      self._mode = BNO055.OPERATION_MODE_NDOF

   def begin(self, mode=None):
      if mode is None: mode = BNO055.OPERATION_MODE_NDOF
      # Open I2C bus
      self._bus = smbus.SMBus(1)

      # Make sure we have the right device
      if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
         sleep(1)  # Wait for the device to boot up
         if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            return False

      # Switch to config mode
      self.setMode(BNO055.OPERATION_MODE_CONFIG)

      # Trigger a reset and wait for the device to boot up again
      self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x20])
      sleep(1)
      while self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
         sleep(0.01)
      sleep(0.05)

      # Set to normal power mode
      self.writeBytes(BNO055.BNO055_PWR_MODE_ADDR, [BNO055.POWER_MODE_NORMAL])
      sleep(0.01)

      self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
      self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0])
      sleep(0.01)

      # Set the requested mode
      self.setMode(mode)
      sleep(0.02)
      return True

   def setMode(self, mode):
      self._mode = mode
      self.writeBytes(BNO055.BNO055_OPR_MODE_ADDR, [self._mode])
      sleep(0.03)

   def setExternalCrystalUse(self, useExternalCrystal = True):
      prevMode = self._mode
      self.setMode(BNO055.OPERATION_MODE_CONFIG)
      sleep(0.025)
      self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
      self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x80] if useExternalCrystal else [0])
      sleep(0.01)
      self.setMode(prevMode)
      sleep(0.02)

   def getSystemStatus(self):
      self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
      (sys_stat, sys_err) = self.readBytes(BNO055.BNO055_SYS_STAT_ADDR, 2)
      self_test = self.readBytes(BNO055.BNO055_SELFTEST_RESULT_ADDR)[0]
      return (sys_stat, self_test, sys_err)

   def getRevInfo(self):
      (accel_rev, mag_rev, gyro_rev) = self.readBytes(BNO055.BNO055_ACCEL_REV_ID_ADDR, 3)
      sw_rev = self.readBytes(BNO055.BNO055_SW_REV_ID_LSB_ADDR, 2)
      sw_rev = sw_rev[0] | sw_rev[1] << 8
      bl_rev = self.readBytes(BNO055.BNO055_BL_REV_ID_ADDR)[0]
      return (accel_rev, mag_rev, gyro_rev, sw_rev, bl_rev)

   def getCalibration(self):
      calData = self.readBytes(BNO055.BNO055_CALIB_STAT_ADDR)[0]
      return (calData >> 6 & 0x03, calData >> 4 & 0x03, calData >> 2 & 0x03, calData & 0x03)

   def getTemp(self):
      return self.readBytes(BNO055.BNO055_TEMP_ADDR)[0]

   def getVector(self, vectorType):
      buf = self.readBytes(vectorType, 6)
      xyz = (struct.unpack('3h', (struct.pack('6B', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]))))
      if vectorType == BNO055.VECTOR_MAGNETOMETER: scalingFactor = 16.0
      elif vectorType == BNO055.VECTOR_GYROSCOPE:  scalingFactor = 900.0
      elif vectorType == BNO055.VECTOR_EULER:      scalingFactor = 16.0
      elif vectorType == BNO055.VECTOR_GRAVITY:    scalingFactor = 100.0
      else:                                        scalingFactor = 1.0
      return tuple([i/scalingFactor for i in xyz])

   def getQuat(self):
      buf = self.readBytes(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
      wxyz = (struct.unpack('4h',(struct.pack('8B', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]))))
      return tuple([i * (1.0 / (1 << 14)) for i in wxyz])

   def readBytes(self, register, numBytes=1):
       return self._bus.read_i2c_block_data(self._address, register, numBytes)

   def writeBytes(self, register, byteVals):
       return self._bus.write_i2c_block_data(self._address, register, byteVals)


if __name__ == '__main__':
    TouchGuiApplication(sys.argv)
