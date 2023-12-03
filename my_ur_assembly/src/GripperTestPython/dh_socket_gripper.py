import dh_client_socket
m_device = dh_client_socket.dh_client_socket()

class dh_socket_gripper(object):
    gripper_ID = 0x01
    def open(self,host_add,port) :
        ret = 0
        ret = m_device.connect_device(host_add, port)
        if(ret < 0) :
            print('open failed')
            return ret
        else :
            print('open successful')
            return ret

    def close() :
        m_device.disconnect_device()

    def WriteRegisterFunc(self,index, value) :
        send_buf = [0 for i in range(14)]
        send_buf[0] = 0xFF;
        send_buf[1] = 0xFE;
        send_buf[2] = 0xFD;
        send_buf[3] = 0xFC;
        send_buf[4] = self.gripper_ID
        send_buf[5] = (index>>8)&0xFF;
        send_buf[6] = index&0xFF;
        send_buf[7] = 0x01;
        send_buf[8] = 0x00;

        send_buf[9] = value&0xFF;
        send_buf[10] = (value>>8)&0xFF;
        send_buf[11] = 0x00;
        send_buf[12] = 0x00;

        send_buf[13] = 0xFB;

        send_temp = send_buf
        ret = False
        retrycount = 3

        while ( ret == False ):
            ret = False

            if(retrycount < 0) :
                break
            retrycount = retrycount - 1

            wdlen = m_device.device_wrire(send_temp)
            if(len(send_temp) != wdlen) :
                print('write error ! write : ', send_temp)
                continue

            rev_buf = m_device.device_read(wdlen)
            if(len(rev_buf) == wdlen) :
                ret = True
        return ret

    def ReadRegisterFunc(self,index) :
        send_buf = [0 for i in range(14)]
        send_buf[0] = 0xFF
        send_buf[1] = 0xFE
        send_buf[2] = 0xFD
        send_buf[3] = 0xFC

        send_buf[4] = self.gripper_ID

        send_buf[5] = (index>>8)&0xFF
        send_buf[6] = index&0xFF
        send_buf[7] = 0x00
        send_buf[8] = 0x00

        send_buf[9] = 0x00
        send_buf[10] = 0x00
        send_buf[11] = 0x00
        send_buf[12] = 0x00

        send_buf[13] = 0xFB

        send_temp = send_buf
        ret = False
        retrycount = 3
        value = -1

        while ( ret == False ):
            ret = False

            if(retrycount < 0) :
                break
            retrycount = retrycount - 1

            wdlen = m_device.device_wrire(send_temp)
            if(len(send_temp) != wdlen) :
                print('write error ! write : ', send_temp)
                continue

            rev_buf = m_device.device_read(wdlen)
            if(len(rev_buf) == wdlen) :
                value = ((rev_buf[9]&0xFF)|(rev_buf[10] << 8))
                ret = True
            #print('read value : ', value)
        return value

    def Initialization(self) :
        self.WriteRegisterFunc(0x0802,0x00)
        
    def SetTargetPosition(self,refpos) :
        self.WriteRegisterFunc(0x0602,refpos);

    def SetTargetForce(self,force) :
        self.WriteRegisterFunc(0x0502,force);
        
    #def SetTargetSpeed(self,speed) :
     #   self.WriteRegisterFunc(0x0104,speed);

    def GetCurrentPosition(self) :
        return self.ReadRegisterFunc(0x0602);

    def GetCurrentTargetForce(self) :
        return self.ReadRegisterFunc(0x0502);

    #def GetCurrentTargetSpeed(self) :
     #   return self.ReadRegisterFunc(0x0104);

    def GetInitState(self) :
        return self.ReadRegisterFunc(0x0802);

    def GetGripState(self) :
        return self.ReadRegisterFunc(0x0F01);

    """description of class"""


