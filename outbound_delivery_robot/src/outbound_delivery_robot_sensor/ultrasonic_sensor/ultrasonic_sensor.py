import RPi.GPIO as GPIO
import time

# GPIO 모드 설정 (BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # GPIO 경고 메시지 비활성화

# GPIO 핀 할당
GPIO_TRIGGER = 23
GPIO_ECHO = 24

# GPIO 핀 방향 설정 (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def measure_distance():
    # 트리거 핀을 HIGH로 설정
    GPIO.output(GPIO_TRIGGER, True)
    # 0.01ms 후 트리거 핀을 LOW로 설정
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    startTime = time.time()
    arrivalTime = time.time()

    # 에코 핀이 LOW에서 HIGH로 변할 때까지 대기
    while GPIO.input(GPIO_ECHO) == 0:
        startTime = time.time()

    # 에코 핀이 HIGH에서 LOW로 변할 때까지 대기
    while GPIO.input(GPIO_ECHO) == 1:
        arrivalTime = time.time()

    # 시간을 계산하여 거리 측정
    timeElapsed = arrivalTime - startTime
    distance = (timeElapsed * 34300) / 2

    return distance

if __name__ == '__main__':
    try:
        while True:
            dist = measure_distance()
            print("Measured distance = %.1f cm" % dist)
            time.sleep(1)

    except KeyboardInterrupt:
        print("Measurement stopped by user")
        GPIO.cleanup()
