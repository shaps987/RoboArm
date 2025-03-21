#Github link: https://github.com/geoffklee/micropython-smbus.git
try:
    from machine import I2C
except ImportError:
    raise ImportError("Can't find the micropython machine.I2C class: "
                      "perhaps you don't need this adapter?")


class SMBus(I2C):
    """ Provides an 'SMBus' module which supports some of the py-smbus
        i2c methods, as well as being a subclass of machine.I2C

        Hopefully this will allow you to run code that was targeted at
        py-smbus unmodified on micropython.

        Use it like you would the machine.I2C class:

            import usmbus.SMBus

            bus = SMBus(id=0, scl=machine.Pin(15), sda=machine.Pin(10), freq=100000)
            bus.read_byte_data(addr, register)
            ... etc
    """

    def read_byte_data(self, addr, register):
        """ Read a single byte from register of device at addr
            Returns a single byte """
        return self.readfrom_mem(addr, register, 1)[0]

    def read_i2c_block_data(self, addr, register, length):
        """ Read a block of length from register of device at addr
            Returns a bytes object filled with whatever was read """
        register_size = 8
        if isinstance(register, list):
            temp = 0
            register_size = 0
            for r in bytes(register):
                temp <<= 8
                temp |= r
                register_size += 8
            register = temp
        return self.readfrom_mem(addr, register, length, addrsize=register_size)

    def write_byte_data(self, addr, register, data):
        """ Write a single byte from buffer data to register of device at addr
            Returns None """
        # writeto_mem() expects something it can treat as a buffer
        if isinstance(data, int):
            data = bytes([data])
        return self.writeto_mem(addr, register, data)

    def write_i2c_block_data(self, addr, register, data):
        """ Write multiple bytes of data to register of device at addr
            Returns None """
        # writeto_mem() expects something it can treat as a buffer
        if not isinstance(data, bytes):
            if not isinstance(data, list):
                data = [data]
            data = bytes(data)

        register_size = 8
        if isinstance(register, list):
            temp = 0
            register_size = 0
            for r in bytes(register):
                temp <<= 8
                temp |= r
                register_size += 8
            register = temp

        return self.writeto_mem(addr, register, data, addrsize=register_size)

    
    def read_byte(self, addr):
        return self.readfrom(addr, 1)[0]

    def write_byte(self, addr, value):
        self.writeto(addr, bytes([value]))
    
    # The following haven't been implemented, but could be.
    def read_word_data(self, *args, **kwargs):
        """ Not yet implemented """
        raise RuntimeError("Not yet implemented")

    def write_word_data(self, *args, **kwargs):
        """ Not yet implemented """
        raise RuntimeError("Not yet implemented")
