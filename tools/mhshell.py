#!/usr/bin/env python

# a text based HUCH shell

from fccom import historyconsole as hc

import sys
import time
import socket
import os
import readline
from optparse import OptionParser

version = 1

def sock_open():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(1)
        sock.settimeout(1.0)
    except socket.error, msg:
        sys.stderr.write("[ERROR] %s\n" % msg[1])
        sys.exit(1)
    return sock

def sock_connect(sock, host, port):
    try:
        sock.connect((host, port))
    except socket.error, msg:
        sys.stderr.write("[ERROR] %s\n" % msg[1])
        sys.exit(2)

def mhsh_print_header():
   print "mavhub shell v{0}".format(version)
   # print "connecting to {0}:{1}".format(uc.rhost, uc.rport)

def mhsh_main(argv):
    # parse arguments
    parser = OptionParser()
    parser.add_option("-H", "--host",
                      dest="mhhost", default="localhost",
                      help="mavhub hostname")
    parser.add_option("-P", "--port",
                      dest="mhport", default=32000,
                      help="mavhub port")
    (options, args) = parser.parse_args()

    # emit configuration
    mhsh_print_header()
    print "Config: %s:%d" % (options.mhhost, int(options.mhport))

    # ready to connect
    sock = sock_open()
    sock_connect(sock, options.mhhost, int(options.mhport))

    # create console
    con = hc.HistoryConsole(histfile=".mhshell-history")

    # set up txbuf
    txbuf = ""

    # enter main loop
    while True:
        # print "main loop"
        # time.sleep(1)
        try:
            txbuf = con.raw_input("cmd: ")
        except EOFError:
            print "EOF reached"
            sys.exit(0)
        if txbuf != "":
            print "Transmitting: ", txbuf
            sock.send(txbuf)
            # set proper timeout
            time.sleep(0.1)
        try:
            print sock.recv(1024)
        except socket.error:
            print "recv timed out"
        # readline.add_history(txbuf)
    
if __name__ == '__main__':
    # for param in os.environ.keys():
    #     print "%20s %s" % (param,os.environ[param])
    sys.exit(mhsh_main(sys.argv))
