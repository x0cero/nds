//! KEY1 (Blowfish-style) secure-area decryption, ported from ndstool.
//! The secure area is the first 2KB of an ARM9 binary that loads at
//! offset 0x4000; clean dumps keep it encrypted (hardware decrypts at
//! boot), so direct boot has to do the same.

const ENCR_DATA: &[u8] = include_bytes!("key1.bin");

struct Key1 {
    magic: [u32; 18 + 1024],
    key: [u32; 3],
}

impl Key1 {
    fn new(gamecode: u32) -> Self {
        let mut magic = [0u32; 18 + 1024];
        for (i, m) in magic.iter_mut().enumerate() {
            *m = u32::from_le_bytes(ENCR_DATA[i * 4..i * 4 + 4].try_into().unwrap());
        }
        let mut k = Self {
            magic,
            key: [gamecode, gamecode >> 1, gamecode << 1],
        };
        k.apply_keycode();
        k.apply_keycode();
        k
    }

    fn lookup(&self, v: u32) -> u32 {
        let a = self.magic[(v >> 24 & 0xFF) as usize + 18];
        let b = self.magic[(v >> 16 & 0xFF) as usize + 18 + 256];
        let c = self.magic[(v >> 8 & 0xFF) as usize + 18 + 512];
        let d = self.magic[(v & 0xFF) as usize + 18 + 768];
        d.wrapping_add(c ^ b.wrapping_add(a))
    }

    fn encrypt(&self, x1: &mut u32, x0: &mut u32) {
        let mut a = *x1;
        let mut b = *x0;
        for i in 0..16 {
            let c = self.magic[i] ^ a;
            a = b ^ self.lookup(c);
            b = c;
        }
        *x0 = a ^ self.magic[16];
        *x1 = b ^ self.magic[17];
    }

    fn decrypt(&self, x1: &mut u32, x0: &mut u32) {
        let mut a = *x1;
        let mut b = *x0;
        for i in (2..18).rev() {
            let c = self.magic[i] ^ a;
            a = b ^ self.lookup(c);
            b = c;
        }
        *x1 = b ^ self.magic[0];
        *x0 = a ^ self.magic[1];
    }

    fn apply_keycode(&mut self) {
        let (mut k1, mut k2) = (self.key[2], self.key[1]);
        self.encrypt(&mut k1, &mut k2);
        self.key[2] = k1;
        self.key[1] = k2;
        let (mut k1, mut k0) = (self.key[1], self.key[0]);
        self.encrypt(&mut k1, &mut k0);
        self.key[1] = k1;
        self.key[0] = k0;
        let keybytes: Vec<u8> = self.key[..2].iter().flat_map(|w| w.to_le_bytes()).collect();
        for j in 0..18 {
            let mut r3 = 0u32;
            for i in 0..4 {
                r3 = r3 << 8 | keybytes[(j * 4 + i) & 7] as u32;
            }
            self.magic[j] ^= r3;
        }
        let mut t1 = 0u32;
        let mut t0 = 0u32;
        for i in (0..18).step_by(2) {
            self.encrypt(&mut t1, &mut t0);
            self.magic[i] = t1;
            self.magic[i + 1] = t0;
        }
        for i in (0..0x400).step_by(2) {
            self.encrypt(&mut t1, &mut t0);
            self.magic[18 + i] = t1;
            self.magic[18 + i + 1] = t0;
        }
    }
}

/// Decrypt a KEY1-encrypted secure area (2KB at ROM offset 0x4000) in
/// place. Returns false (leaving data untouched) if the "encryObj"
/// check fails, i.e. the area was already decrypted or is corrupt.
pub fn decrypt_secure_area(gamecode: u32, sec: &mut [u8]) -> bool {
    let word = |d: &[u8], i: usize| u32::from_le_bytes(d[i * 4..i * 4 + 4].try_into().unwrap());
    if word(sec, 0) == 0xE7FF_DEFF {
        return false; // already decrypted
    }
    let mut buf: Vec<u32> = (0..0x200).map(|i| word(sec, i)).collect();

    let mut k = Key1::new(gamecode);
    let (mut w1, mut w0) = (buf[1], buf[0]);
    k.decrypt(&mut w1, &mut w0);
    buf[1] = w1;
    buf[0] = w0;
    k.key[1] <<= 1;
    k.key[2] >>= 1;
    k.apply_keycode();
    let (mut w1, mut w0) = (buf[1], buf[0]);
    k.decrypt(&mut w1, &mut w0);

    if w0 != 0x72636E65 || w1 != 0x6A624F79 {
        eprintln!(
            "secure area: decrypt check failed (w0={:#010X} w1={:#010X}, raw={:#010X} {:#010X})",
            w0, w1, word(sec, 0), word(sec, 1)
        );
        return false; // not "encryObj": corrupt secure area
    }
    buf[0] = 0xE7FF_DEFF;
    buf[1] = 0xE7FF_DEFF;
    for i in (2..0x200).step_by(2) {
        let (mut w1, mut w0) = (buf[i + 1], buf[i]);
        k.decrypt(&mut w1, &mut w0);
        buf[i + 1] = w1;
        buf[i] = w0;
    }
    for (i, w) in buf.iter().enumerate() {
        sec[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }
    true
}
