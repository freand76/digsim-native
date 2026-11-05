// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! The levels of the combinational cells, like a compiled simulator (Verilator) orders
//! its logic: a cell gets a rank above the ranks of all cells that drive it. A settle
//! evaluates the cells that changed inputs once each, in the order of their ranks,
//! without events in between. The cells in a combinational loop (and a cell that drives
//! itself) don't get a rank, they keep their events and delays.

/// No rank: the cell isn't combinational or is in a loop
pub const NONE: u32 = u32::MAX;

/// The ranks of the cells, from the cells each cell drives (`edges[cell]`) and which cells
/// can have a rank (`candidate[cell]`)
pub fn ranks(edges: &[Vec<u32>], candidate: &[bool]) -> Vec<u32> {
    let count = edges.len();
    let in_loop = loops(edges, candidate);
    let ranked: Vec<bool> = (0..count).map(|c| candidate[c] && !in_loop[c]).collect();
    // The longest path from a cell without ranked drivers (Kahn's algorithm)
    let mut drivers = vec![0u32; count];
    for (cell, targets) in edges.iter().enumerate() {
        if !ranked[cell] {
            continue;
        }
        for &target in targets {
            if ranked[target as usize] {
                drivers[target as usize] += 1;
            }
        }
    }
    let mut rank = vec![NONE; count];
    let mut ready: Vec<u32> = Vec::new();
    for cell in 0..count {
        if ranked[cell] && drivers[cell] == 0 {
            rank[cell] = 0;
            ready.push(cell as u32);
        }
    }
    while let Some(cell) = ready.pop() {
        let next = rank[cell as usize] + 1;
        for &target in &edges[cell as usize] {
            let t = target as usize;
            if !ranked[t] {
                continue;
            }
            if rank[t] == NONE || rank[t] < next {
                rank[t] = next;
            }
            drivers[t] -= 1;
            if drivers[t] == 0 {
                ready.push(target);
            }
        }
    }
    rank
}

/// The candidate cells in a loop of candidates (Tarjan's strongly connected components,
/// without recursion: a gate netlist can be deep)
fn loops(edges: &[Vec<u32>], candidate: &[bool]) -> Vec<bool> {
    let count = edges.len();
    let mut index = vec![u32::MAX; count];
    let mut low = vec![0u32; count];
    let mut on_stack = vec![false; count];
    let mut stack: Vec<u32> = Vec::new();
    let mut in_loop = vec![false; count];
    let mut next_index = 0u32;
    // (cell, the next of its edges to look at)
    let mut work: Vec<(u32, usize)> = Vec::new();
    for root in 0..count {
        if !candidate[root] || index[root] != u32::MAX {
            continue;
        }
        work.push((root as u32, 0));
        index[root] = next_index;
        low[root] = next_index;
        next_index += 1;
        stack.push(root as u32);
        on_stack[root] = true;
        while let Some(&mut (cell, ref mut position)) = work.last_mut() {
            let c = cell as usize;
            if *position < edges[c].len() {
                let target = edges[c][*position] as usize;
                *position += 1;
                if !candidate[target] {
                    continue;
                }
                if target == c {
                    in_loop[c] = true;
                } else if index[target] == u32::MAX {
                    index[target] = next_index;
                    low[target] = next_index;
                    next_index += 1;
                    stack.push(target as u32);
                    on_stack[target] = true;
                    work.push((target as u32, 0));
                } else if on_stack[target] {
                    low[c] = low[c].min(index[target]);
                }
                continue;
            }
            work.pop();
            if let Some(&(parent, _)) = work.last() {
                let p = parent as usize;
                low[p] = low[p].min(low[c]);
            }
            if low[c] == index[c] {
                let mut members = Vec::new();
                loop {
                    let member = stack.pop().unwrap();
                    on_stack[member as usize] = false;
                    members.push(member);
                    if member == cell {
                        break;
                    }
                }
                if members.len() > 1 {
                    for member in members {
                        in_loop[member as usize] = true;
                    }
                }
            }
        }
    }
    in_loop
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chain_and_loop() {
        // 0 -> 1 -> 2, 3 <-> 4 -> 5, 6 -> 6
        let edges = vec![
            vec![1],
            vec![2],
            vec![],
            vec![4],
            vec![3, 5],
            vec![],
            vec![6],
        ];
        let candidate = vec![true; 7];
        let rank = ranks(&edges, &candidate);
        assert_eq!(rank, vec![0, 1, 2, NONE, NONE, 0, NONE]);
    }
}
