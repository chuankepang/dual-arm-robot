import sys
import socket
from signal import signal, SIGPIPE, SIG_DFL

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

class dh_client_socket(object):

    def connect_device(self, host_add, port) :
        ret = -1
        
        ret = client_socket.connect_ex((host_add,port))
        print('Connect recv: ',ret)
        if(ret < 0) :
            print('Connect Error')
            ret = -1

        else :
            print('Connect Success')
            ret = 0
        return ret

    def disconnect_device() :
        client_socket.close()
        return True

    def device_wrire(self, nDate) :
        length = 0
        date = bytes(nDate)

	signal(SIGPIPE, SIG_DFL)

        length = client_socket.send(date)
        #print('length: ', length)
        return length

    def device_read(self, length) :

	signal(SIGPIPE, SIG_DFL)

        date = client_socket.recv(length)
        #print('recv: ', date.hex())
        return date


    """description of class"""


