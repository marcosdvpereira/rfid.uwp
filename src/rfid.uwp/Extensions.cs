using Windows.Devices.Spi;

namespace rfid.uwp
{
    static class Extensions
    {

        public static byte Transfer(this SpiDevice spi, byte value)
        {
            var buffer = new byte[1];

            spi.TransferFullDuplex(new byte[] { value }, buffer);

            return buffer[0];
        }

    }
}
