// Copyright 2025 Soheil E.nia
#pragma once
//
// =====================================================================================
//  Time-to-goal reachability prune  ("skip useless nodes")            [REVERTIBLE]
// =====================================================================================
//
//  In the kinodynamic state spaces (R2T, Dubins-time, Thruster) the LAST coordinate is
//  "time-to-goal" tau(x). Two invariants hold:
//
//    (I1)  steer() rejects any edge unless tau(from) - tau(to) > 0, i.e. tau strictly
//          DECREASES along every feasible edge toward the goal.
//    (I2)  T_robot (the robot's remaining time-to-goal budget) is MONOTONICALLY
//          NON-INCREASING over the mission (it counts down to 0).
//
//  Define the relevant set  R(T) = { x : tau(x) <= T }.  By (I1), R(T) is closed under
//  "successor toward goal": for any x in R(T_robot) the optimal cost-to-goal of x AND an
//  optimal path realizing it lie entirely inside R(T_robot). Hence a node with
//  tau > T_robot can never lie on any feasible path the robot can still execute, and
//  deleting V \ R(T_robot) from the cost-to-goal computation changes neither the optimal
//  cost nor the optimal path of any relevant node (in particular the robot's anchor).
//
//  => Pruning such nodes is an EXACT reachability prune, not an approximation. It does NOT
//     break asymptotic optimality: it only avoids computing provably-irrelevant values.
//     The argument is independent of the cost objective (time / fuel / length).
//
//  This master switch makes the whole optimization revertible. Set it to 0 and every
//  prune site below compiles to `false`, so the planners behave EXACTLY as if the
//  optimization never existed (useful for A/B benchmarking AO/quality vs. speed).
//
//  CAUTION: the prune is sound ONLY while (I1) and (I2) hold. If a future feature lets
//  T_robot increase (budget reset / re-localization to an earlier time), or a state space
//  is added where the last coordinate is not strictly-monotone time, this must be revisited.
//
// TODO: I need to list the invariants that allows me to use the time cone prune!!!
/*
  I was a bit skeptical on what happens to new sampled node when we are not orphaning a subtree due to time cone! would the new samples connect to the dead nodes! NO! because:

Yes — after reading the whole flow carefully, the key point is: the dead nodes you skip are not becoming INF; they are simply irrelevant because no live new sample can adopt them as parent in your backward-search time ordering. The protection is coming from the sampling/remapping geometry plus the parent-direction invariant, not from orphaning them.
Why a dead node cannot parent a live new node

In your planner, parent is always closer to the goal and has smaller time-to-goal than the child; equivalently, tau increases as you move away from the root/goal. Your new sample is first remapped into the reachable cone with remapTimeToGoalCone(..., T_robot, ...), so any kept new node has tau(new) <= T_robot. A dead existing node has tau(dead) > T_robot, hence tau(dead) > tau(new), which means it is farther from the goal than the new node and therefore cannot be a valid parent of the new node in your backward search.

That is exactly why this parent search is safe even though it only filters on finite LMC:

cpp
for (auto& [neighbor, edge_info] : node->forwardNeighbors()) {
    if (neighbor->getLMC() == INF) continue;
    ...
}

A dead neighbor may still have finite LMC, but if statespace_->steer(sample_val, neighbor->getStateValue()) respects your backward-search feasibility rule, then that edge is in the child-to-parent direction only when neighbor is closer to goal, i.e. only when tau(neighbor) < tau(new). For a dead neighbor, that inequality cannot hold because tau(dead) > T_robot >= tau(new). So such a dead node never even enters forwardNeighbors() as a valid parent candidate.
Where the real guard is

The guard is earlier than the LMC test: it is in the combination of:

    remapTimeToGoalCone(...) restricting new nodes to the live cone, and

    statespace_->steer(sample_val, neighbor) only returning is_valid when the direction is dynamically/time feasible.

So for a live new node:

    live parent candidate: possible, if tau(parent) < tau(new),

    dead node as parent: impossible, because tau(dead) > T_robot >= tau(new).

That means the non-orphaned dead nodes can remain in the KD-tree with stale finite LMC, but they are only harmless geometric bystanders for new-node parent selection.
What dead nodes can still do

They can still appear in KD-tree radius/KNN results, and they can still receive incoming/backward edges in bookkeeping if you do not prune them there, but your explicit prune in deferred incoming validation already skips the expensive reverse steer for dead neighbors:

cpp
if (TIME_CONE_PRUNED(neighbor, T_robot)) continue;

So even the “dead node adopts new node” direction is blocked for the dead region.

The one place where dead nodes still matter is obstacle handling: if they were already connected long ago and later become dead only because T_robot advanced, they may keep finite LMC/G. That is okay because your plan() loop pops and discards them before expansion:

cpp
if (TIME_CONE_PRUNED(z, T_robot)) {
    v_open_heap_.pop();
    continue;
}

So they do not broadcast cost into the live cone.
The exact condition this relies on

This whole argument is only correct if your statespace_->steer(a,b) in kinodynamic mode enforces the backward-search temporal ordering, meaning a valid edge from a to b implies tau(b) < tau(a) for the new-node parent-search direction. If steer() can sometimes return a valid trajectory even when time ordering is reversed or ignored, then a dead finite-LMC node really could be chosen as parent, and then you would need an explicit TTG filter in candidate-parent construction:

cpp
if (TIME_CONE_PRUNED(neighbor, T_robot)) continue;

But from everything you explained so far, your planner invariant is that steer feasibility already enforces this ordering.
Bottom line for FMTX

So your concern was good, but in this code the answer is: they do not need to become INF for new-node attachment to stay correct. The reason is not “dead nodes have infinite cost”; the real reason is “dead nodes are on the wrong side of the time ordering to ever be feasible parents for a live sampled node.”

If you want, the safest belt-and-suspenders change is still to add this inside eager parent collection:

cpp
if (TIME_CONE_PRUNED(neighbor, T_robot)) continue;

It should be redundant under your invariant, but it makes the assumption explicit and protects you if steer() ever becomes less strict later.

*/

#ifndef ENABLE_TIME_CONE_PRUNE
#define ENABLE_TIME_CONE_PRUNE 0
#endif

// Predicate: is `node` outside the robot's remaining reachable cone (budget `t_budget`)?
// `is_geometric_mode_` is a member of every kinodynamic planner; in geometric mode there
// is no time coordinate so the prune is disabled. `t_budget` is passed explicitly so the
// caller can use the live member T_robot OR a function-local budget.
#if ENABLE_TIME_CONE_PRUNE
  #define TIME_CONE_PRUNED(node, t_budget) (!is_geometric_mode_ && (node)->getTimeToGoal() > (t_budget))
#else
  #define TIME_CONE_PRUNED(node, t_budget) (false)
#endif
