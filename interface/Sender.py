import json
import os
import configparser
import zmq
import time
import ad_interface_functions


class ZMQSender:

    def __init__(self,
                 theme: str):

        """Class to construct a ZMQ Sender.

        :param theme: Theme the sender should publish information to.

        :Authors:
            Alexander Heilmeier\n
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.11.2020
        """

        # --------------------------------------------------------------------------------------------------------------
        # IMPORT INTERFACE CONFIG PARAMETERS ---------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        repo_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        parser = configparser.ConfigParser()
        pars = {}

        if not parser.read(os.path.join(repo_path, "interface/params/interface_config.ini")):
            raise ValueError('Specified config file does not exist or is empty!')

        pars["receiver_exp_zmq"] = json.loads(parser.get('INTERFACE_SPEC_SENDER', theme))

        # --------------------------------------------------------------------------------------------------------------
        # OPEN INTERFACES ----------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # initialization -----------------------------------------------------------------------------------------------
        zmq_context = zmq.Context()
        int_receiver_zmq = {"opts_exp": pars["receiver_exp_zmq"]}

        # RECEIVER via ZMQ ---------------------------------------------------------------------------------------------
        int_receiver_zmq["sock_exp"] = zmq_context.socket(zmq.PUB)
        int_receiver_zmq["sock_exp"].bind("tcp://*:%s" % int_receiver_zmq["opts_exp"]["port"])

        # wait a short time until all sockets are really bound (ZMQ specific problem) ----------------------------------
        time.sleep(0.5)

        self.int_receiver_zmq = int_receiver_zmq

        print("All sockets opened (ESIM sender)!")

    def send(self,
             data):
        """Send messages.

        :param data: The data to be sent.

        :Authors:
            Alexander Heilmeier\n
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.11.2020
        """

        # --------------------------------------------------------------------------------------------------------------
        # SEND MESSAGES ------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # LTPL via ZMQ -------------------------------------------------------------------------------------------------
        ad_interface_functions.zmq_export.zmq_export(sock=self.int_receiver_zmq["sock_exp"],
                                                     topic=self.int_receiver_zmq["opts_exp"]["topic"],
                                                     data=data,
                                                     datatype='pyobj')

    def __del__(self):
        """Delete sender object.

        :Authors:
            Alexander Heilmeier\n
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.11.2020
        """

        # --------------------------------------------------------------------------------------------------------------
        # CLOSE SOCKETS ------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        self.int_receiver_zmq["sock_exp"].close()

        time.sleep(0.5)

        print("All sockets closed (ESIM sender)!")


if __name__ == "__main__":
    """Function to check the functionality of the ZMQ sender."""
    
    zs = ZMQSender(theme='receiver_exp_vplanner')

    while True:
        zs.send(data=True)
