/*--------------------------------------------------------------------
Copyright 2020 fukuen

Mic rec/play demo is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This software is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with This software.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <Arduino.h>
#include <Sipeed_ST7789.h>
#include <SD.h>
#include "uarths.h"
#include "utility/Button.h"
#include "gpio.h"
#include "lcd.h"
#include "audio_mic.h"
#include "audio_speaker.h"

#define WIDTH 240
#define HEIGHT 240
#define AXP173_ADDR 0x34
#define FRAME_LEN 512
#define SAMPLING 16000
#define FILE_NAME "/RECDATA.WAV"
int16_t rx_buf[FRAME_LEN];
int16_t tx_buf[FRAME_LEN];

SPIClass spi_0(SPI0); // MUST be SPI0 for Maix series on board LCD
Sipeed_ST7789 lcd(WIDTH, HEIGHT, spi_0);

#define DEBOUNCE_MS 10
#define KEY_UP      11 // MaixCube
#define KEY_PRESS   10
#define KEY_DOWN    16
#define LED_R       13
#define LED_G       12
#define LED_B       14
Button BtnA = Button(KEY_UP, true, DEBOUNCE_MS);
Button BtnB = Button(KEY_PRESS, true, DEBOUNCE_MS);
Button BtnC = Button(KEY_DOWN, true, DEBOUNCE_MS);

typedef enum _rec_play_mode
{
    MODE_PLAY = 0,
    MODE_REC = 1,
    MODE_STOP = 2,
    MODE_PLAYING = 3,
    MODE_RECORDING = 4
} rec_play_mode_t;

rec_play_mode_t rec_play_mode = MODE_STOP;

File file;

int pio_init() {
    // led
    fpioa_set_function(LED_R, FUNC_GPIO0);
    gpio_set_drive_mode(0, GPIO_DM_OUTPUT);
    gpio_set_pin(0, GPIO_PV_HIGH);

    fpioa_set_function(LED_G, FUNC_GPIO1);
    gpio_set_drive_mode(1, GPIO_DM_OUTPUT);
    gpio_set_pin(1, GPIO_PV_HIGH);

    fpioa_set_function(LED_B, FUNC_GPIO2);
    gpio_set_drive_mode(2, GPIO_DM_OUTPUT);
    gpio_set_pin(2, GPIO_PV_HIGH);

    return 0;
}

void drawMenu() {
    lcd.fillScreen(COLOR_BLACK);
    lcd.fillRect(0, 0, WIDTH, 20, COLOR_CYAN);
    lcd.setCursor(1, 1);
    lcd.setTextSize(2);
    lcd.setTextColor(COLOR_BLACK);
    lcd.println("REC/PLAY demo v0.1");
    lcd.setCursor(0, 16);
    lcd.println("");
    lcd.setTextColor(COLOR_WHITE);
    for (int i = 0; i < 10; i++) {
        lcd.println("");
    }
    lcd.println("<-: REC/STOP");
    lcd.println("->: PLAY/STOP");
    lcd.setTextColor(COLOR_GREENYELLOW);
    lcd.print("Copyright 2020 fukuen");
}

void drawClear() {
    lcd.fillRect(0, 32, WIDTH, 144, COLOR_BLACK);
    lcd.setCursor(36, 96);
    lcd.setTextSize(2);
    lcd.setTextColor(COLOR_WHITE);
    gpio_set_pin(0, GPIO_PV_HIGH);
    gpio_set_pin(1, GPIO_PV_HIGH);
}

void drawStop() {
    drawClear();
    lcd.println("    STOP");
}

void drawPlaying() {
    drawClear();
    lcd.println(" PLAYING...");
    gpio_set_pin(1, GPIO_PV_LOW);
}

void drawRecording() {
    drawClear();
    lcd.println("RECORDING...");
    gpio_set_pin(0, GPIO_PV_LOW);
}

void writeSD(int offset) {
    file.write((uint8_t *)&rx_buf, FRAME_LEN * 2);
}

void dispReg() {
    uint8_t reg = 0;
    int res = es8374_read_reg(0x6e, &reg); //flag
    Serial.printf("reg %u\n", reg);
}

void axp173_init()
{
    Wire.begin((uint8_t) SDA, (uint8_t) SCL, 400000);
    Wire.beginTransmission(AXP173_ADDR);
    int err = Wire.endTransmission();
    if (err) {
        Serial.printf("Power management ic not found.\n");
        return;
    }
    Serial.printf("AXP173 found.\n");
    // Clear the interrupts
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x46);
    Wire.write(0xFF);
    Wire.endTransmission();
    // set target voltage and current of battery(axp173 datasheet PG.)
    // charge current (default)780mA -> 190mA
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x33);
    Wire.write(0xC1);
    Wire.endTransmission();
    // REG 10H: EXTEN & DC-DC2 control
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.requestFrom(AXP173_ADDR, 1, 1);
    int reg = Wire.read();
    Wire.beginTransmission(AXP173_ADDR);
    Wire.write(0x10);
    Wire.write(reg & 0xFC);
    Wire.endTransmission();
}

void setup() {
    pll_init();
    plic_init();
    dmac_init();
    uarths_init();
    Serial.begin(115200);
    axp173_init();

    lcd.begin(15000000, COLOR_BLACK);
    lcd.setRotation(2); // 
    tft_write_command(INVERSION_DISPALY_ON);

    if (!SD.begin()){
        Serial.printf( "SD mount failed.\n" );
        lcd.println("SD mount failed.");
        while (true) {}
    }

    pio_init();
    audio_mic_init();
    audio_mic_set_buffer(rx_buf, FRAME_LEN);
    audio_mic_set_sample_rate(SAMPLING);
    audio_speaker_init();
    audio_speaker_set_buffer(tx_buf, FRAME_LEN);
    audio_speaker_set_sample_rate(SAMPLING);

//    es8374_write_reg(0x1e, 0xA4); // speaker 5db
    es8374_write_reg(0x1e, 0xA7); // speaker 7.5db

    /* Enable the machine interrupt */
    sysctl_enable_irq();

    drawMenu();
}

void loop() {
    BtnA.read();
    BtnB.read();
    BtnC.read();

    if (BtnC.wasPressed()) {
        Serial.printf("Button C pressed.\n");
        if (rec_play_mode == MODE_STOP) {
            // stop -> playing
            file = SD.open(FILE_NAME);
            if (file) {
                Serial.printf("File open success.\n");
                rec_play_mode = MODE_PLAYING;
                drawPlaying();
            } else {
                Serial.printf("File open failed.\n");
            }
        } else if (rec_play_mode == MODE_PLAYING) {
            // playing -> stop
            audio_speaker_stop();
            file.close();
            rec_play_mode = MODE_STOP;
            drawStop();
        }
    }

    if (BtnA.wasPressed()) {
        Serial.printf("Button A pressed.\n");
        if (rec_play_mode == MODE_STOP) {
            // stop -> recording
            try {
                SD.remove(FILE_NAME);
            } catch (char *str) {
                Serial.printf("%s\n", str);
            }
            file = SD.open(FILE_NAME, FILE_WRITE);
            if (file) {
                Serial.printf("File open success.\n");
                audio_mic_start();
                drawRecording();
                rec_play_mode = MODE_RECORDING;
            } else {
                Serial.printf("File open failed.\n");
            }
        } else if (rec_play_mode == MODE_RECORDING) {
            // recording -> stop
            audio_mic_stop();
            file.close();
            rec_play_mode = MODE_STOP;
            drawStop();
        }
    }

    if (rec_play_mode == MODE_RECORDING) {
        if (audio_mic_get_state() == RECV_STATE_RECVING) {
            //
//        } else if (audio_mic_get_state() == RECV_STATE_READY) {
        } else {
            writeSD(0);
            audio_mic_clear();
        }
    } else if (rec_play_mode == MODE_PLAYING) {
        if (audio_speaker_get_state() != PLAY_STATE_PLAYING) {
            size_t len = file.readBytes((uint8_t *)tx_buf, FRAME_LEN * 2);
            if (len > 0) {
                audio_speaker_play();
            } else {
                audio_speaker_stop();
                file.close();
                rec_play_mode = MODE_STOP;
                drawStop();
            }
        }
    }
}
