"""
This python script is used to configure the DW1000 chip as an anchor for ranging functionalities. It must be used in conjunction with the RangingTAG script. 
It requires the following modules: DW1000, DW1000Constants and monotonic.
"""


import DW1000
import monotonic
import DW1000Constants as C
import RPi.GPIO as GPIO

new_trans_flag = 1
anchors = 25
lastActivity = 0
lastMsg = 0
expectedMsgId = C.POLL
protocolFailed = False
sentAck = False
receivedAck = False
LEN_DATA = 16
data = [0] * LEN_DATA
timePollAckSentTS = 0
timePollAckReceivedTS = 0
timePollReceivedTS = 0
timeRangeReceivedTS = 0
timePollSentTS = 0
timeRangeSentTS = 0
timeComputedRangeTS = 0
REPLY_DELAY_TIME_US = 7000 
# The polling range frequency defines the time interval between every distance poll in milliseconds. Feel free to change its value. 
MSG_RANGE_FREQ = 100 # the distance between the tag and the anchor will be estimated every second.

def millis():
    """
    This function returns the value (in milliseconds) of a clock which never goes backwards. It detects the inactivity of the chip and
    is used to avoid having the chip stuck in an undesirable state.
    """    
    return int(round(monotonic.monotonic() * C.MILLISECONDS))


def handleSent():
    """
    This is a callback called from the module's interrupt handler when a transmission was successful. 
    It sets the sentAck variable as True so the loop can continue.
    """            
    global sentAck
    sentAck = True


def handleReceived():
    """
    This is a callback called from the module's interrupt handler when a reception was successful. 
    It sets the received receivedAck as True so the loop can continue.
    """       
    global receivedAck
    receivedAck = True


def noteActivity():
    """
    This function records the time of the last activity so we can know if the device is inactive or not.
    """        
    global lastActivity
    lastActivity = millis()


def resetInactive():
    """
    This function restarts the default polling operation when the device is deemed inactive.
    """    
    global expectedMsgId
    print("reset inactive")    
    expectedMsgId = C.POLL
    receiver()
    noteActivity()

def transmitAnchorMsg(anchor):
    """
    This function sends the read anchor message to known anchors
    """        
    global data, lastMsg
    while (millis() - lastMsg < MSG_RANGE_FREQ):
        pass
    DW1000.newTransmit()
    data[0] = anchor
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()
    lastMsg = millis()

def transmitPollAck():
    """
    This function sends the polling acknowledge message which is used to confirm the reception of the polling message. 
    """        
    global data
    DW1000.newTransmit()
    data[0] = C.POLL_ACK
    DW1000.setDelay(REPLY_DELAY_TIME_US, C.MICROSECONDS)
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()


def transmitRangeAcknowledge():
    """
    This functions sends the range acknowledge message which tells the tag that the ranging function was successful and another ranging transmission can begin.
    """
    global data
    DW1000.newTransmit()
    data[0] = C.RANGE_REPORT
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()


def transmitRangeFailed():
    """
    This functions sends the range failed message which tells the tag that the ranging function has failed and to start another ranging transmission.
    """    
    global data
    DW1000.newTransmit()
    data[0] = C.RANGE_FAILED
    DW1000.setData(data, LEN_DATA)
    DW1000.startTransmit()


def receiver():
    """
    This function configures the chip to prepare for a message reception.
    """    
    global data
    DW1000.newReceive()
    DW1000.receivePermanently()
    DW1000.startReceive()


def computeRangeAsymmetric():
    """
    This is the function which calculates the timestamp used to determine the range between the devices.
    """
    global timeComputedRangeTS
    round1 = DW1000.wrapTimestamp(timePollAckReceivedTS - timePollSentTS)
    reply1 = DW1000.wrapTimestamp(timePollAckSentTS - timePollReceivedTS)
    round2 = DW1000.wrapTimestamp(timeRangeReceivedTS - timePollAckSentTS)
    reply2 = DW1000.wrapTimestamp(timeRangeSentTS - timePollAckReceivedTS)
    timeComputedRangeTS = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2)


def loop():
    global sentAck, receivedAck, timePollAckSentTS, timePollReceivedTS, timePollSentTS, timePollAckReceivedTS, timeRangeReceivedTS, protocolFailed, data, expectedMsgId, timeRangeSentTS, \
        anchors, new_trans_flag

    if new_trans_flag:
        new_trans_flag = 0
        transmitAnchorMsg(anchors)
        noteActivity()
    if (sentAck == False and receivedAck == False):
        if ((millis() - lastActivity) > C.RESET_PERIOD):
            # initial receiver
            # resetInactive()
            new_trans_flag = 1
        return

    if sentAck:
        sentAck = False
        msgId = data[0]
        if msgId == 25:
            print ("An25 Msg Sented")
        if msgId == C.POLL_ACK:
            print ("POLL ACK Sented")
            timePollAckSentTS = DW1000.getTransmitTimestamp()
            noteActivity()

    if receivedAck:
        receivedAck = False
        data = DW1000.getData(LEN_DATA)
        msgId = data[0]
        # if msgId != expectedMsgId:
        #     protocolFailed = True
        if msgId == C.POLL:
            print ("POLL Rcved")
            protocolFailed = False
            timePollReceivedTS = DW1000.getReceiveTimestamp()
        #     expectedMsgId = C.RANGE
            # transmit POLL_ACK
            transmitPollAck()
            noteActivity()
        elif msgId == C.RANGE:
            timeRangeReceivedTS = DW1000.getReceiveTimestamp()
            print ("RANGE Rcved")
        #     expectedMsgId = C.POLL
            if protocolFailed == False:
                timePollSentTS = DW1000.getTimeStamp(data, 1)
                timePollAckReceivedTS = DW1000.getTimeStamp(data, 6)
                timeRangeSentTS = DW1000.getTimeStamp(data, 11)
                computeRangeAsymmetric()
                # Transmit RANGE_REPORT
                transmitRangeAcknowledge()
                distance = (timeComputedRangeTS % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO
                print("Distance: %.2f m" %(distance))
                new_trans_flag = 1
            else:
                new_trans_flag = 1
        #         # transmit RANGE_FAILED
        #         transmitRangeFailed()

        #     noteActivity()



try: 
    PIN_RST = 17   
    PIN_IRQ = 19
    PIN_SS = 27
    GPIO.setmode(GPIO.BCM)
    # GPIO.setup(PIN_RST, GPIO.IN)
    DW1000.begin(PIN_IRQ)
    DW1000.setup(PIN_SS)
    print("DW1000 initialized")
    print("############### ANCHOR ##############")

    DW1000.generalConfiguration("82:17:5B:D5:A9:9A:E2:9C", C.MODE_LONGDATA_RANGE_ACCURACY)
    DW1000.registerCallback("handleSent", handleSent)
    DW1000.registerCallback("handleReceived", handleReceived)
    DW1000.setAntennaDelay(C.ANTENNA_DELAY_RASPI)

    receiver()
    # transmitPoll()
    noteActivity()
    while 1:
        loop()

except KeyboardInterrupt:
    DW1000.close()
