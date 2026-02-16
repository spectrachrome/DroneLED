//! Post-processing effects for the LED buffer.
//!
//! Effects are represented as enum variants and applied in sequence to the
//! pixel buffer via [`apply_pipeline`]. Everything is zero-allocation, fully
//! static, and `Copy`.

use smart_leds::RGB8;

/// Red channel gamma 2.6 correction look-up table.
const GAMMA_R: [u8; 256] = [
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,
      1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,
      3,  3,  4,  4,  4,  4,  5,  5,  5,  5,  5,  6,  6,  6,  6,  7,
      7,  7,  8,  8,  8,  9,  9,  9, 10, 10, 10, 11, 11, 11, 12, 12,
     13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20,
     20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29,
     30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 38, 38, 39, 40, 41, 42,
     42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
     58, 59, 60, 61, 62, 63, 64, 65, 66, 68, 69, 70, 71, 72, 73, 75,
     76, 77, 78, 80, 81, 82, 84, 85, 86, 88, 89, 90, 92, 93, 94, 96,
     97, 99,100,102,103,105,106,108,109,111,112,114,115,117,119,120,
    122,124,125,127,129,130,132,134,136,137,139,141,143,145,146,148,
    150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,
    182,184,186,188,191,193,195,197,199,202,204,206,209,211,213,215,
    218,220,223,225,227,230,232,235,237,240,242,245,247,250,252,255,
];

/// Green channel gamma 2.7 correction look-up table.
///
/// Slightly steeper curve than red to compensate for the human eye's higher
/// sensitivity to green and the typically higher efficiency of green WS2812B
/// dies, without crushing low-end values too aggressively.
const GAMMA_G: [u8; 256] = [
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,
      1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  3,  3,
      3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,  5,  6,  6,
      6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10, 10, 10, 11,
     11, 12, 12, 12, 13, 13, 14, 14, 14, 15, 15, 16, 16, 17, 17, 18,
     18, 19, 19, 20, 20, 21, 21, 22, 23, 23, 24, 24, 25, 26, 26, 27,
     28, 28, 29, 30, 30, 31, 32, 33, 33, 34, 35, 36, 36, 37, 38, 39,
     40, 41, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 51, 52, 53,
     55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 68, 69, 70, 71,
     72, 74, 75, 76, 77, 79, 80, 81, 83, 84, 85, 87, 88, 89, 91, 92,
     94, 95, 97, 98,100,101,103,104,106,107,109,110,112,114,115,117,
    119,120,122,124,125,127,129,131,132,134,136,138,140,141,143,145,
    147,149,151,153,155,157,159,161,163,165,167,169,171,173,175,178,
    180,182,184,186,188,191,193,195,198,200,202,205,207,209,212,214,
    216,219,221,224,226,229,231,234,237,239,242,244,247,250,252,255,
];

/// Blue channel gamma 2.5 correction look-up table.
const GAMMA_B: [u8; 256] = [
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
      1,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  4,  4,
      4,  4,  4,  5,  5,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,
      8,  8,  9,  9,  9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14,
     14, 15, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 22,
     22, 23, 23, 24, 25, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32,
     33, 33, 34, 35, 36, 36, 37, 38, 39, 40, 40, 41, 42, 43, 44, 45,
     46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
     61, 62, 63, 64, 65, 67, 68, 69, 70, 71, 72, 73, 75, 76, 77, 78,
     80, 81, 82, 83, 85, 86, 87, 89, 90, 91, 93, 94, 95, 97, 98, 99,
    101,102,104,105,107,108,110,111,113,114,116,117,119,121,122,124,
    125,127,129,130,132,134,135,137,139,141,142,144,146,148,150,151,
    153,155,157,159,161,163,165,166,168,170,172,174,176,178,180,182,
    184,186,189,191,193,195,197,199,201,204,206,208,210,212,215,217,
    219,221,224,226,228,231,233,235,238,240,243,245,248,250,253,255,
];

/// A single post-processing effect applied to the LED buffer.
#[derive(Clone, Copy)]
pub enum PostEffect {
    /// Scale every channel by `brightness / 256`.
    Brightness(u8),
    /// Clamp total strip current to `max_ma` milliamps.
    CurrentLimit { max_ma: u32 },
    /// Per-channel gamma correction via separate R/G/B look-up tables.
    ///
    /// Red γ=2.6, Green γ=3.0, Blue γ=2.5. The steeper green curve
    /// compensates for the human eye's higher sensitivity to green and
    /// the typically higher efficiency of green WS2812B dies.
    Gamma,
    /// Per-channel color balance (max scaling per channel, 0–255).
    ///
    /// Compensates for unequal LED die efficiency and human eye sensitivity.
    /// Typical WS2812B defaults: R=255, G=176, B=240.
    ColorBalance { r: u8, g: u8, b: u8 },
}

impl PostEffect {
    /// Apply this effect to the LED buffer in-place.
    pub fn apply(&self, leds: &mut [RGB8]) {
        match self {
            PostEffect::Brightness(n) => {
                let n = *n as u16;
                for px in leds.iter_mut() {
                    px.r = (px.r as u16 * n / 256) as u8;
                    px.g = (px.g as u16 * n / 256) as u8;
                    px.b = (px.b as u16 * n / 256) as u8;
                }
            }
            PostEffect::CurrentLimit { max_ma } => {
                // Model: each WS2812B draws ~1 mA idle + 8 mA per colour
                // channel at full PWM.
                let channel_sum: u32 = leds
                    .iter()
                    .map(|c| c.r as u32 + c.g as u32 + c.b as u32)
                    .sum();

                let num = leds.len() as u32;
                // total_ma = idle + channel_draw
                //   idle         = num_leds * 1 mA
                //   channel_draw = channel_sum * 8 / 255 mA
                let total_ma_x255 = num * 255 + channel_sum * 8;

                let budget_x255 = *max_ma * 255;
                if total_ma_x255 > budget_x255 {
                    // Scale factor = budget / total  (both ×255, cancels)
                    for px in leds.iter_mut() {
                        px.r = (px.r as u32 * budget_x255 / total_ma_x255) as u8;
                        px.g = (px.g as u32 * budget_x255 / total_ma_x255) as u8;
                        px.b = (px.b as u32 * budget_x255 / total_ma_x255) as u8;
                    }
                }
            }
            PostEffect::Gamma => {
                for px in leds.iter_mut() {
                    px.r = GAMMA_R[px.r as usize];
                    px.g = GAMMA_G[px.g as usize];
                    px.b = GAMMA_B[px.b as usize];
                }
            }
            PostEffect::ColorBalance { r, g, b } => {
                let r = *r as u16;
                let g = *g as u16;
                let b = *b as u16;
                for px in leds.iter_mut() {
                    px.r = (px.r as u16 * r / 255) as u8;
                    px.g = (px.g as u16 * g / 255) as u8;
                    px.b = (px.b as u16 * b / 255) as u8;
                }
            }
        }
    }
}

/// Apply a sequence of post-processing effects to the LED buffer.
pub fn apply_pipeline(leds: &mut [RGB8], effects: &[PostEffect]) {
    for effect in effects {
        effect.apply(leds);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brightness_scales_channels() {
        let mut leds = [RGB8 { r: 200, g: 100, b: 50 }];
        PostEffect::Brightness(128).apply(&mut leds);
        assert_eq!(leds[0].r, 100);
        assert_eq!(leds[0].g, 50);
        assert_eq!(leds[0].b, 25);
    }

    #[test]
    fn brightness_zero_blacks_out() {
        let mut leds = [RGB8 { r: 255, g: 255, b: 255 }];
        PostEffect::Brightness(0).apply(&mut leds);
        assert_eq!(leds[0], RGB8 { r: 0, g: 0, b: 0 });
    }

    #[test]
    fn gamma_uses_per_channel_luts() {
        let mut leds = [RGB8 { r: 128, g: 128, b: 128 }];
        PostEffect::Gamma.apply(&mut leds);
        // Per-channel LUTs differ, so outputs should differ
        assert_eq!(leds[0].r, GAMMA_R[128]);
        assert_eq!(leds[0].g, GAMMA_G[128]);
        assert_eq!(leds[0].b, GAMMA_B[128]);
        // Green gamma is steeper (3.0 vs 2.6), so green output < red output
        assert!(leds[0].g < leds[0].r);
    }

    #[test]
    fn gamma_endpoints() {
        let mut leds = [RGB8 { r: 0, g: 0, b: 0 }, RGB8 { r: 255, g: 255, b: 255 }];
        PostEffect::Gamma.apply(&mut leds);
        assert_eq!(leds[0], RGB8 { r: 0, g: 0, b: 0 });
        assert_eq!(leds[1], RGB8 { r: 255, g: 255, b: 255 });
    }

    #[test]
    fn color_balance_scales_per_channel() {
        let mut leds = [RGB8 { r: 255, g: 255, b: 255 }];
        PostEffect::ColorBalance { r: 255, g: 128, b: 200 }.apply(&mut leds);
        assert_eq!(leds[0].r, 255);
        assert_eq!(leds[0].g, 128);
        assert_eq!(leds[0].b, 200);
    }

    #[test]
    fn current_limit_under_budget_is_noop() {
        let mut leds = [RGB8 { r: 10, g: 10, b: 10 }; 2];
        let before = leds;
        PostEffect::CurrentLimit { max_ma: 5000 }.apply(&mut leds);
        assert_eq!(leds, before);
    }

    #[test]
    fn current_limit_over_budget_reduces() {
        let mut leds = [RGB8 { r: 255, g: 255, b: 255 }; 100];
        PostEffect::CurrentLimit { max_ma: 500 }.apply(&mut leds);
        // All pixels should be reduced
        assert!(leds[0].r < 255);
    }

    #[test]
    fn pipeline_applies_in_order() {
        let mut leds = [RGB8 { r: 200, g: 100, b: 50 }];
        apply_pipeline(
            &mut leds,
            &[PostEffect::Brightness(128), PostEffect::Gamma],
        );
        // Brightness first: 200*128/256=100, then gamma_r LUT[100]
        assert_eq!(leds[0].r, GAMMA_R[100]);
    }
}
