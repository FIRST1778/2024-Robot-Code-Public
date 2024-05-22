package org.chillout1778.subsystems

import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.commands.lights.LEDCommand

//if shooterState == ready, make white
//if shooterState == stored but not ready, light blue
//if shooterState == empty, make dark blue

// Adafruit DotStars are addressable SK9822 LEDs controlled via an SPI
// interface.
// The LEDs which are farthest from the source have their frames
// emitted first.

// DotStar manual:
//     https://cdn-learn.adafruit.com/downloads/pdf/adafruit-dotstar-leds.pdf
// The APA102 datasheet (which SK9822s are compatible with):
//     https://cdn-learn.adafruit.com/assets/assets/000/084/592/original/APA102_White_LED.pdf?1574117639
// A helpful blog post for the protocol:
//     https://cpldcpu.wordpress.com/2014/11/30/understanding-the-apa102-superled/

object Lights: SubsystemBase() {
    private val spi = SPI(SPI.Port.kOnboardCS0).apply { setClockRate(4_000_000) }

    init{
        defaultCommand = LEDCommand()
    }

    private fun emit(frame: ByteArray) {
        spi.write(frame, frame.size)
    }

    private fun startFrame() =
            // The start frame is at least 4 0x00 bytes.
            ByteArray(4) { 0x00.toByte() }

    private fun ledFrame(r: Byte, g: Byte, b: Byte): ByteArray {
        return byteArrayOf(
            // The first 3 bits (111) begin an LED frame.  The next 5 bits
            // are the brightness, which we always set to full (31 out of 31).
            (0xE0 + 10).toByte(),
            // Blue-green-red for our LEDs.
            b, g, r
        )
    }

    private fun endFrame() =
            // The end frame is defined by the datasheet to be 4 0xFF bytes, but
            // the actual requirement seems to be a 1 bit for every two LEDs in
            // the chain, ergo at least one 0xFF byte for every 16 LEDs.
            ByteArray((Constants.Lights.LED_COUNT + 15) / 16) { 0xFF.toByte() }

    fun write(r: Int, g: Int, b: Int) {
        emit(startFrame())
        repeat(Constants.Lights.LED_COUNT) {
            emit(ledFrame(r.toByte(),g.toByte(),b.toByte()))
        }
        emit(endFrame())
    }
}
