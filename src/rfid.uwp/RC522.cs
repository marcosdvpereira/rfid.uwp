using System;
using System.Threading.Tasks;

using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;
using Windows.Devices.Spi;

namespace rfid.uwp
{
    public class RC522
    {

        private const int MAX_LEN = 16;

        private const byte PCD_IDLE = 0x00;
        private const byte PCD_AUTHENT = 0x0E;
        private const byte PCD_RECEIVE = 0x08;
        private const byte PCD_TRANSMIT = 0x04;
        private const byte PCD_TRANSCEIVE = 0x0C;
        private const byte PCD_RESETPHASE = 0x0F;
        private const byte PCD_CALCCRC = 0x03;

        private const byte PICC_REQIDL = 0x26;
        private const byte PICC_REQALL = 0x52;
        private const byte PICC_ANTICOLL = 0x93;
        private const byte PICC_SElECTTAG = 0x93;
        private const byte PICC_AUTHENT1A = 0x60;
        private const byte PICC_AUTHENT1B = 0x61;
        private const byte PICC_READ = 0x30;
        private const byte PICC_WRITE = 0xA0;
        private const byte PICC_DECREMENT = 0xC0;
        private const byte PICC_INCREMENT = 0xC1;
        private const byte PICC_RESTORE = 0xC2;
        private const byte PICC_TRANSFER = 0xB0;
        private const byte PICC_HALT = 0x50;

        private const int MI_OK = 0;
        private const int MI_NOTAGERR = 1;
        private const int MI_ERR = 2;

        private const byte Reserved00 = 0x00;
        private const byte CommandReg = 0x01;
        private const byte CommIEnReg = 0x02;
        private const byte DivlEnReg = 0x03;
        private const byte CommIrqReg = 0x04;
        private const byte DivIrqReg = 0x05;
        private const byte ErrorReg = 0x06;
        private const byte Status1Reg = 0x07;
        private const byte Status2Reg = 0x08;
        private const byte FIFODataReg = 0x09;
        private const byte FIFOLevelReg = 0x0A;
        private const byte WaterLevelReg = 0x0B;
        private const byte ControlReg = 0x0C;
        private const byte BitFramingReg = 0x0D;
        private const byte CollReg = 0x0E;
        private const byte Reserved01 = 0x0F;

        private const byte Reserved10 = 0x10;
        private const byte ModeReg = 0x11;
        private const byte TxModeReg = 0x12;
        private const byte RxModeReg = 0x13;
        private const byte TxControlReg = 0x14;
        private const byte TxAutoReg = 0x15;
        private const byte TxSelReg = 0x16;
        private const byte RxSelReg = 0x17;
        private const byte RxThresholdReg = 0x18;
        private const byte DemodReg = 0x19;
        private const byte Reserved11 = 0x1A;
        private const byte Reserved12 = 0x1B;
        private const byte MifareReg = 0x1C;
        private const byte Reserved13 = 0x1D;
        private const byte Reserved14 = 0x1E;
        private const byte SerialSpeedReg = 0x1F;

        private const byte Reserved20 = 0x20;
        private const byte CRCResultRegM = 0x21;
        private const byte CRCResultRegL = 0x22;
        private const byte Reserved21 = 0x23;
        private const byte ModWidthReg = 0x24;
        private const byte Reserved22 = 0x25;
        private const byte RFCfgReg = 0x26;
        private const byte GsNReg = 0x27;
        private const byte CWGsPReg = 0x28;
        private const byte ModGsPReg = 0x29;
        private const byte TModeReg = 0x2A;
        private const byte TPrescalerReg = 0x2B;
        private const byte TReloadRegH = 0x2C;
        private const byte TReloadRegL = 0x2D;
        private const byte TCounterValueRegH = 0x2E;
        private const byte TCounterValueRegL = 0x2F;

        private const byte Reserved30 = 0x30;
        private const byte TestSel1Reg = 0x31;
        private const byte TestSel2Reg = 0x32;
        private const byte TestPinEnReg = 0x33;
        private const byte TestPinValueReg = 0x34;
        private const byte TestBusReg = 0x35;
        private const byte AutoTestReg = 0x36;
        private const byte VersionReg = 0x37;
        private const byte AnalogTestReg = 0x38;
        private const byte TestDAC1Reg = 0x39;
        private const byte TestDAC2Reg = 0x3A;
        private const byte TestADCReg = 0x3B;
        private const byte Reserved31 = 0x3C;
        private const byte Reserved32 = 0x3D;
        private const byte Reserved33 = 0x3E;
        private const byte Reserved34 = 0x3F;

        private byte[] serNum = new byte[5];

        private SpiDevice spi;
        private GpioPin gpioReset;
        private GpioPin gpioData;

        private int _chipSelectPin;
        private int _resetPowerDownPin;
        private int _dataCommandPin;

        private void InitGpio()
        {
            var ioController = GpioController.GetDefault();

            gpioData = ioController.OpenPin(_dataCommandPin);
            gpioData.SetDriveMode(GpioPinDriveMode.Output);
            gpioData.Write(GpioPinValue.High);

            gpioReset = ioController.OpenPin(_resetPowerDownPin);
            gpioReset.SetDriveMode(GpioPinDriveMode.Output);
            gpioReset.Write(GpioPinValue.Low);
        }

        private async Task InitSpi()
        {
            var settings = new SpiConnectionSettings(_chipSelectPin);
            settings.Mode = SpiMode.Mode0;

            var spiAqs = SpiDevice.GetDeviceSelector();
            var devicesInfo = await DeviceInformation.FindAllAsync(spiAqs);
            spi = await SpiDevice.FromIdAsync(devicesInfo[0].Id, settings);
        }

        private void init()
        {
            gpioData.Write(GpioPinValue.High);

            reset();

            writeMFRC522(TModeReg, 0x8D);
            writeMFRC522(TPrescalerReg, 0x3E);
            writeMFRC522(TReloadRegL, 30);
            writeMFRC522(TReloadRegH, 0);
            writeMFRC522(TxAutoReg, 0x40);
            writeMFRC522(ModeReg, 0x3D);

            antennaOn();
        }

        #region API

        public async void InitAll(int chipSelectPin, int resetPowerDownPin, int dataCommandPin)
        {
            _chipSelectPin = chipSelectPin;
            _resetPowerDownPin = resetPowerDownPin;
            _dataCommandPin = dataCommandPin;

            InitGpio();
            await InitSpi();

            init();
        }

        public unsafe bool isCard()
        {
            byte status;
            byte[] str = new byte[MAX_LEN];

            fixed (byte* s = str)
                status = MFRC522Request(PICC_REQIDL, s);

            return (status == MI_OK);
        }

        public unsafe bool readCardSerial()
        {

            byte status;
            byte[] str = new byte[MAX_LEN];

            fixed (byte* s = str)
                status = anticoll(s);

            Array.Copy(serNum, str, 5);

            return (status == MI_OK);
        }

        public void reset()
        {
            writeMFRC522(CommandReg, PCD_RESETPHASE);
        }

        public void writeMFRC522(byte addr, byte val)
        {
            gpioData.Write(GpioPinValue.Low);

            spi.Transfer((byte)((addr << 1) & 0x7E));
            spi.Transfer(val);

            gpioData.Write(GpioPinValue.High);
        }

        public byte readMFRC522(byte addr)
        {
            byte val;
            gpioData.Write(GpioPinValue.Low);
            spi.Transfer((byte)(((addr << 1) & 0x7E) | 0x80));
            val = spi.Transfer(0x00);
            gpioData.Write(GpioPinValue.High);
            return val;
        }

        public void setBitMask(byte reg, byte mask)
        {
            byte tmp;
            tmp = readMFRC522(reg);
            writeMFRC522(reg, (byte)(tmp | mask));
        }

        public void clearBitMask(byte reg, byte mask)
        {
            byte tmp;
            tmp = readMFRC522(reg);
            writeMFRC522(reg, (byte)(tmp & (~mask)));
        }

        public void antennaOn()
        {
            byte temp;

            temp = readMFRC522(TxControlReg);
            if ((temp & 0x03) == 0)
                setBitMask(TxControlReg, 0x03);
        }

        public void antennaOff()
        {
            byte temp;

            temp = readMFRC522(TxControlReg);
            if ((temp & 0x03) == 0)
                clearBitMask(TxControlReg, 0x03);
        }

        public unsafe void calculateCRC(byte* pIndata, byte len, byte* pOutData)
        {
            byte i, n;

            clearBitMask(DivIrqReg, 0x04);
            setBitMask(FIFOLevelReg, 0x80);

            for (i = 0; i < len; i++)
                writeMFRC522(FIFODataReg, *(pIndata + i));
            writeMFRC522(CommandReg, PCD_CALCCRC);

            i = 0xFF;
            do
            {
                n = readMFRC522(DivIrqReg);
                i--;
            }
            while ((i != 0) && (n & 0x04) == 0);

            pOutData[0] = readMFRC522(CRCResultRegL);
            pOutData[1] = readMFRC522(CRCResultRegM);
        }

        public unsafe byte MFRC522ToCard(byte command, byte* sendData, byte sendLen, byte* backData, uint* backLen)
        {
            byte status = MI_ERR;
            byte irqEn = 0x00;
            byte waitIRq = 0x00;
            byte lastBits;
            byte n;
            uint i;

            switch (command)
            {
                case PCD_AUTHENT:
                    {
                        irqEn = 0x12;
                        waitIRq = 0x10;
                        break;
                    }
                case PCD_TRANSCEIVE:
                    {
                        irqEn = 0x77;
                        waitIRq = 0x30;
                        break;
                    }
                default:
                    break;
            }

            writeMFRC522(CommIEnReg, (byte)(irqEn | 0x80));
            clearBitMask(CommIrqReg, 0x80);
            setBitMask(FIFOLevelReg, 0x80);

            writeMFRC522(CommandReg, PCD_IDLE);

            for (i = 0; i < sendLen; i++)
                writeMFRC522(FIFODataReg, sendData[i]);

            writeMFRC522(CommandReg, command);
            if (command == PCD_TRANSCEIVE)
                setBitMask(BitFramingReg, 0x80);

            i = 2000;
            do
            {
                n = readMFRC522(CommIrqReg);
                i--;
            }
            while ((i != 0) && (n & 0x01) == 0 && (n & waitIRq) == 0);

            clearBitMask(BitFramingReg, 0x80);

            if (i != 0)
            {
                if ((readMFRC522(ErrorReg) & 0x1B) == 0)
                {
                    status = MI_OK;
                    if ((n & irqEn & 0x01) != 0)
                        status = MI_NOTAGERR;

                    if (command == PCD_TRANSCEIVE)
                    {
                        n = readMFRC522(FIFOLevelReg);
                        lastBits = (byte)(readMFRC522(ControlReg) & 0x07);
                        if (lastBits != 0)
                            *backLen = (uint)((n - 1) * 8 + lastBits);
                        else
                            *backLen = (uint)(n * 8);

                        if (n == 0)
                            n = 1;
                        if (n > MAX_LEN)
                            n = MAX_LEN;

                        for (i = 0; i < n; i++)
                            backData[i] = readMFRC522(FIFODataReg);
                    }
                }
                else
                    status = MI_ERR;
            }

            return status;
        }

        public unsafe byte MFRC522Request(byte reqMode, byte* TagType)
        {
            byte status;
            uint backBits;

            writeMFRC522(BitFramingReg, 0x07);

            TagType[0] = reqMode;
            status = MFRC522ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

            if ((status != MI_OK) || (backBits != 0x10))
                status = MI_ERR;

            return status;
        }

        public unsafe byte anticoll(byte* serNum)
        {
            byte status;
            byte i;
            byte serNumCheck = 0;
            uint unLen;

            writeMFRC522(BitFramingReg, 0x00);

            serNum[0] = PICC_ANTICOLL;
            serNum[1] = 0x20;
            status = MFRC522ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

            if (status == MI_OK)
            {
                for (i = 0; i < 4; i++)
                    serNumCheck ^= serNum[i];
                if (serNumCheck != serNum[i])
                    status = MI_ERR;
            }

            return status;
        }

        public unsafe byte auth(byte authMode, byte BlockAddr, byte* Sectorkey, byte* serNum)
        {
            byte status;
            uint recvBits;
            byte i;
            byte[] buff = new byte[12];

            buff[0] = authMode;
            buff[1] = BlockAddr;
            for (i = 0; i < 6; i++)
                buff[i + 2] = *(Sectorkey + i);
            for (i = 0; i < 4; i++)
                buff[i + 8] = *(serNum + i);

            fixed (byte* b = buff)
                status = MFRC522ToCard(PCD_AUTHENT, b, 12, b, &recvBits);

            if ((status != MI_OK) || ((readMFRC522(Status2Reg) & 0x08) == 0))
                status = MI_ERR;

            return status;
        }

        public unsafe byte read(byte blockAddr, byte* recvData)
        {
            byte status;
            uint unLen;

            recvData[0] = PICC_READ;
            recvData[1] = blockAddr;
            calculateCRC(recvData, 2, &recvData[2]);
            status = MFRC522ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

            if ((status != MI_OK) || (unLen != 0x90))
                status = MI_ERR;

            return status;
        }

        public unsafe byte write(byte blockAddr, byte* writeData)
        {
            byte status;
            uint recvBits;
            byte i;
            byte[] buff = new byte[18];

            buff[0] = PICC_WRITE;
            buff[1] = blockAddr;

            fixed (byte* b = buff)
            {
                fixed (byte* buff2 = &buff[2])
                    calculateCRC(b, 2, buff2);

                status = MFRC522ToCard(PCD_TRANSCEIVE, b, 4, b, &recvBits);

                if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
                    status = MI_ERR;

                if (status == MI_OK)
                {
                    for (i = 0; i < 16; i++)
                        buff[i] = *(writeData + i);

                    fixed (byte* buff16 = &buff[16])
                        calculateCRC(b, 16, buff16);

                    status = MFRC522ToCard(PCD_TRANSCEIVE, b, 18, b, &recvBits);

                    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
                        status = MI_ERR;
                }
            }

            return status;
        }

        public unsafe byte selectTag(byte* serNum)
        {
            byte i;
            byte status;
            byte size;
            uint recvBits;
            byte[] buffer = new byte[9];

            buffer[0] = PICC_SElECTTAG;
            buffer[1] = 0x70;

            for (i = 0; i < 5; i++)
                buffer[i + 2] = *(serNum + i);

            fixed (byte* b = buffer)
            {
                fixed (byte* buffer7 = &buffer[7])
                    calculateCRC(b, 7, buffer7);

                status = MFRC522ToCard(PCD_TRANSCEIVE, b, 9, b, &recvBits);
            }

            if ((status == MI_OK) && (recvBits == 0x18))
            {
                size = buffer[0];
            }
            else
            {
                size = 0;
            }

            return size;
        }

        public unsafe void halt()
        {
            byte status;
            uint unLen;
            byte[] buff = new byte[4];

            buff[0] = PICC_HALT;
            buff[1] = 0;

            fixed (byte* b = buff)
            {
                fixed (byte* buff2 = &buff[2])
                    calculateCRC(b, 2, buff2);

                status = MFRC522ToCard(PCD_TRANSCEIVE, b, 4, b, &unLen);
            }
        }

        #endregion

    }
}
