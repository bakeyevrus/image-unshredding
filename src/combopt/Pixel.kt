package combopt

import java.util.*

class Pixel(r: Int, g: Int, b: Int) {

    val colors = arrayOf(r, g, b)
    override fun toString(): String {
        return "Pixel(colors=${Arrays.toString(colors)})"
    }


}