#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import pyaudio
import numpy as np
import scipy.signal

class SoundDirectionPublisher:
    def __init__(self):
        rospy.init_node('sound_direction_publisher', anonymous=True)
        self.pub = rospy.Publisher('sound_direction', Float32, queue_size=10)
        
        # Audio params
        self.RATE = 44100
        self.CHANNELS = 2
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.MIC_DISTANCE = 0.1 # Meters, adjust as needed
        self.SOUND_SPEED = 343.0 # m/s

        self.p = pyaudio.PyAudio()
        try:
            self.stream = self.p.open(format=self.FORMAT,
                                      channels=self.CHANNELS,
                                      rate=self.RATE,
                                      input=True,
                                      frames_per_buffer=self.CHUNK)
            rospy.loginfo("Audio stream opened successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to open audio stream: {e}")
            return

    def gcc_phat(self, sig, refsig, fs=1, max_tau=None, interp=16):
        '''
        This function computes the offset between the signal sig and the reference signal refsig
        using the Generalized Cross Correlation - Phase Transform (GCC-PHAT)method.
        '''
        # make sure the length for the FFT is larger or equal than len(sig) + len(refsig)
        n = sig.shape[0] + refsig.shape[0]

        # Generalized Cross Correlation Phase Transform
        SIG = np.fft.rfft(sig, n=n)
        REFSIG = np.fft.rfft(refsig, n=n)
        R = SIG * np.conj(REFSIG)

        cc = np.fft.irfft(R / np.abs(R), n=(interp * n))

        max_shift = int(interp * n / 2)
        if max_tau:
            max_shift = np.minimum(int(interp * fs * max_tau), max_shift)

        cc = np.concatenate((cc[-max_shift:], cc[:max_shift+1]))

        # find max cross correlation index
        shift = np.argmax(np.abs(cc)) - max_shift

        tau = shift / float(interp * fs)
        
        return tau, cc

    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            try:
                data = self.stream.read(self.CHUNK, exception_on_overflow=False)
                audio_data = np.frombuffer(data, dtype=np.int16)
                
                # Separate channels
                left = audio_data[0::2]
                right = audio_data[1::2]
                
                # Calculate TDOA
                tau, _ = self.gcc_phat(left, right, fs=self.RATE)
                
                # Calculate angle
                # tau = d * sin(theta) / c
                # theta = asin(tau * c / d)
                val = tau * self.SOUND_SPEED / self.MIC_DISTANCE
                val = np.clip(val, -1.0, 1.0) # Clip to valid range for asin
                theta = np.arcsin(val)
                
                degree = np.degrees(theta)
                
                print(degree)
                
                self.pub.publish(degree)
                # rospy.loginfo(f"Direction: {degree:.2f} degrees")
                
            except Exception as e:
                rospy.logwarn(f"Error processing audio: {e}")
            
            rate.sleep()

        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

if __name__ == '__main__':
    try:
        node = SoundDirectionPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
