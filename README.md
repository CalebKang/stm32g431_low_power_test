Timer sync test

## PPM
PPM(Part Par Million)은 1/1,000,000이라는 의미입니다.


예) ±100PPM = ±100/1,000,000 = ±1/10,000이 됩니다.
   1일과 1시간의 설정(시간 정도)을 초단위로 계산하면 다음과 같이 됩니다.

- 1일은 24시간 × 60분 × 60초(초)
   따라서, 1일 오차는
   ±100PPM × 24 × 60 × 60
   86,400/10,000 ≒ 8.6초

- 1시간은 60분 × 60초(초)
   따라서, 1시간의 오차는
   ±100PPM × 60 × 60 = 0.36초


## RTC Alarm
30분에 2~3msec 오차발생
32.768Hz = 0.000 030517578125


## HSE  + TIM << 정확도 Best Solution
30분에 1msec 차이발생
1시간에 6msec 차이 발생


## LPTIM + LSE(1 Div, 32.768Hz)
02:08 : 48164 - 48136 = 0.00091552734375 sec
