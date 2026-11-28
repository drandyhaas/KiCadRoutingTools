//! Tiled dense refcount grid: a drop-in store for the obstacle map's
//! per-cell reference counts (`blocked_cells`, `blocked_vias`, via rungs).
//!
//! The obstacle map used `FxHashMap<u64, u16>` per layer. Building a
//! board-global map stamps millions of cells, and every stamp paid a hash
//! probe (plus rehash growth). Copper is spatially clustered, so a grid of
//! lazily allocated 64 x 64 tiles holds the same counts with one hash probe
//! per TILE (and none at all while consecutive cells stay in the last tile
//! touched, which is the common case for the column spans the rasterizer
//! emits). Semantics are exactly the map's: a cell is present iff its count
//! is nonzero, `len()` is the number of present cells, and iteration visits
//! every present cell once (in tile order; no caller depends on order, the
//! one exported list is sorted by its caller).

use rustc_hash::FxHashMap;

const SHIFT: i32 = 6;
const SIDE: i32 = 1 << SHIFT;
const MASK: i32 = SIDE - 1;
const CELLS: usize = (SIDE * SIDE) as usize;
const NONE: u32 = u32::MAX;

#[inline]
fn split(gx: i32, gy: i32) -> (u64, usize) {
    let tx = (gx >> SHIFT) as u32 as u64;
    let ty = (gy >> SHIFT) as u32 as u64;
    // Column-major inside a tile: the rasterizer emits spans along gy.
    let off = (((gx & MASK) << SHIFT) | (gy & MASK)) as usize;
    ((tx << 32) | ty, off)
}

#[derive(Clone)]
pub struct CountGrid {
    index: FxHashMap<u64, u32>,
    keys: Vec<u64>,
    tiles: Vec<Box<[u16; CELLS]>>,
    nonzero: Vec<u32>,
    len: usize,
    last_key: u64,
    last_idx: u32,
}

impl Default for CountGrid {
    fn default() -> Self {
        Self {
            index: FxHashMap::default(),
            keys: Vec::new(),
            tiles: Vec::new(),
            nonzero: Vec::new(),
            len: 0,
            last_key: 0,
            last_idx: NONE,
        }
    }
}

impl CountGrid {
    #[inline]
    fn find(&self, tkey: u64) -> Option<usize> {
        if self.last_idx != NONE && self.last_key == tkey {
            return Some(self.last_idx as usize);
        }
        self.index.get(&tkey).map(|&i| i as usize)
    }

    #[inline]
    fn find_mut(&mut self, tkey: u64) -> Option<usize> {
        if self.last_idx != NONE && self.last_key == tkey {
            return Some(self.last_idx as usize);
        }
        let i = *self.index.get(&tkey)?;
        self.last_key = tkey;
        self.last_idx = i;
        Some(i as usize)
    }

    #[inline]
    fn find_or_alloc(&mut self, tkey: u64) -> usize {
        if let Some(i) = self.find_mut(tkey) {
            return i;
        }
        let i = self.tiles.len() as u32;
        self.tiles.push(Box::new([0u16; CELLS]));
        self.keys.push(tkey);
        self.nonzero.push(0);
        self.index.insert(tkey, i);
        self.last_key = tkey;
        self.last_idx = i;
        i as usize
    }

    /// Count at (gx, gy); 0 when absent.
    #[inline]
    pub fn get(&self, gx: i32, gy: i32) -> u16 {
        let (tkey, off) = split(gx, gy);
        match self.find(tkey) {
            Some(i) => self.tiles[i][off],
            None => 0,
        }
    }

    #[inline]
    pub fn contains(&self, gx: i32, gy: i32) -> bool {
        self.get(gx, gy) != 0
    }

    /// Add `n` to the count (wrapping, as the map's release-mode `+=` did).
    /// Returns true on the absent -> present transition.
    #[inline]
    pub fn add(&mut self, gx: i32, gy: i32, n: u16) -> bool {
        let (tkey, off) = split(gx, gy);
        let i = self.find_or_alloc(tkey);
        let c = &mut self.tiles[i][off];
        let old = *c;
        let new = old.wrapping_add(n);
        *c = new;
        if old == 0 && new != 0 {
            self.nonzero[i] += 1;
            self.len += 1;
            true
        } else {
            if old != 0 && new == 0 {
                self.nonzero[i] -= 1;
                self.len -= 1;
            }
            false
        }
    }

    #[inline]
    pub fn inc(&mut self, gx: i32, gy: i32) -> bool {
        self.add(gx, gy, 1)
    }

    /// Decrement a present count; returns true on the present -> absent
    /// transition. An absent cell is left untouched (the map's `get_mut`
    /// miss).
    #[inline]
    pub fn dec(&mut self, gx: i32, gy: i32) -> bool {
        let (tkey, off) = split(gx, gy);
        let i = match self.find_mut(tkey) {
            Some(i) => i,
            None => return false,
        };
        let c = &mut self.tiles[i][off];
        match *c {
            0 => false,
            1 => {
                *c = 0;
                self.nonzero[i] -= 1;
                self.len -= 1;
                true
            }
            _ => {
                *c -= 1;
                false
            }
        }
    }

    /// Subtract `n` from a present count, saturating at removal (the map's
    /// `if *cnt > n { *cnt -= n } else { remove }`). Returns true on the
    /// present -> absent transition; an absent cell is left untouched.
    #[inline]
    pub fn sub(&mut self, gx: i32, gy: i32, n: u16) -> bool {
        let (tkey, off) = split(gx, gy);
        let i = match self.find_mut(tkey) {
            Some(i) => i,
            None => return false,
        };
        let c = &mut self.tiles[i][off];
        if *c == 0 {
            false
        } else if *c > n {
            *c -= n;
            false
        } else {
            *c = 0;
            self.nonzero[i] -= 1;
            self.len -= 1;
            true
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn clear(&mut self) {
        *self = Self::default();
    }

    /// Drop tiles that hold no present cell, then shrink the containers.
    pub fn shrink_to_fit(&mut self) {
        if self.nonzero.iter().any(|&n| n == 0) {
            let mut out = Self::default();
            for (gx, gy, c) in self.iter() {
                out.add(gx, gy, c);
            }
            *self = out;
        }
        self.index.shrink_to_fit();
        self.keys.shrink_to_fit();
        self.tiles.shrink_to_fit();
        self.nonzero.shrink_to_fit();
    }

    /// Every present cell as (gx, gy, count).
    pub fn iter(&self) -> impl Iterator<Item = (i32, i32, u16)> + '_ {
        self.tiles.iter().enumerate().filter(move |(i, _)| self.nonzero[*i] != 0).flat_map(
            move |(i, t)| {
                let k = self.keys[i];
                let bx = ((k >> 32) as u32 as i32) << SHIFT;
                let by = (k as u32 as i32) << SHIFT;
                t.iter().enumerate().filter(|(_, &c)| c != 0).map(move |(off, &c)| {
                    let off = off as i32;
                    (bx + (off >> SHIFT), by + (off & MASK), c)
                })
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustc_hash::FxHashMap;

    fn lcg(s: &mut u64) -> u64 {
        *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        *s >> 33
    }

    /// Randomised equivalence against the hash map it replaces, including
    /// negative coordinates and tile borders.
    #[test]
    fn matches_hashmap_refcounts() {
        let mut g = CountGrid::default();
        let mut m: FxHashMap<(i32, i32), u16> = FxHashMap::default();
        let mut s = 7u64;
        for step in 0..200_000 {
            let gx = (lcg(&mut s) % 300) as i32 - 150;
            let gy = (lcg(&mut s) % 300) as i32 - 150;
            if lcg(&mut s) % 3 == 0 {
                let was = m.get(&(gx, gy)).copied().unwrap_or(0);
                let cleared = g.dec(gx, gy);
                if was > 1 {
                    *m.get_mut(&(gx, gy)).unwrap() -= 1;
                } else if was == 1 {
                    m.remove(&(gx, gy));
                }
                assert_eq!(cleared, was == 1);
            } else {
                let fresh = g.inc(gx, gy);
                let c = m.entry((gx, gy)).or_insert(0);
                *c += 1;
                assert_eq!(fresh, *c == 1);
            }
            assert_eq!(g.get(gx, gy), m.get(&(gx, gy)).copied().unwrap_or(0));
            if step % 50_000 == 0 {
                g.shrink_to_fit();
            }
        }
        assert_eq!(g.len(), m.len());
        let mut a: Vec<_> = g.iter().collect();
        let mut b: Vec<_> = m.iter().map(|(&(x, y), &c)| (x, y, c)).collect();
        a.sort_unstable();
        b.sort_unstable();
        assert_eq!(a, b);
        let c = g.clone();
        assert_eq!(c.len(), g.len());
    }

    /// `sub` saturates at removal like the map's subtract, and keeps len
    /// and iteration consistent.
    #[test]
    fn sub_matches_hashmap_saturation() {
        let mut g = CountGrid::default();
        let mut m: FxHashMap<(i32, i32), u16> = FxHashMap::default();
        let mut s = 11u64;
        for _ in 0..100_000 {
            let gx = (lcg(&mut s) % 200) as i32 - 100;
            let gy = (lcg(&mut s) % 200) as i32 - 100;
            let n = (lcg(&mut s) % 4) as u16 + 1;
            if lcg(&mut s) % 2 == 0 {
                g.add(gx, gy, n);
                *m.entry((gx, gy)).or_insert(0) += n;
            } else {
                let was = m.get(&(gx, gy)).copied().unwrap_or(0);
                let cleared = g.sub(gx, gy, n);
                if was > n {
                    *m.get_mut(&(gx, gy)).unwrap() -= n;
                } else if was > 0 {
                    m.remove(&(gx, gy));
                }
                assert_eq!(cleared, was > 0 && was <= n);
            }
            assert_eq!(g.get(gx, gy), m.get(&(gx, gy)).copied().unwrap_or(0));
        }
        assert_eq!(g.len(), m.len());
        let mut a: Vec<_> = g.iter().collect();
        let mut b: Vec<_> = m.iter().map(|(&(x, y), &c)| (x, y, c)).collect();
        a.sort_unstable();
        b.sort_unstable();
        assert_eq!(a, b);
    }
}
