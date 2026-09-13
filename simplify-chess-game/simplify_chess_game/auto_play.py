"""Auto-play chess without manual control: python-chess generates the moves.

Each ply runs the same deterministic pipeline as the manual CLI:
HOME -> route(source) -> reverse -> route(target) -> reverse, with
PRE_CLOSE / CLOSE / RELEASE gripper steps. Only quiet moves (no capture,
no promotion, no castling) are chosen so no discard handling is needed.
"""
from __future__ import annotations

import argparse
import random

import chess
import rclpy

from .chess_executor import ChessExecutor
from .trajectory_executor import ExecutionError


def pick_move(board: chess.Board, rng: random.Random) -> chess.Move:
    quiet = [m for m in board.legal_moves
             if not board.is_capture(m) and m.promotion is None
             and not board.is_castling(m)]
    if quiet:
        return rng.choice(quiet)
    # Late game may have only captures/promotions left; allow anything
    # except promotions (route replay cannot change piece shape).
    fallback = [m for m in board.legal_moves if m.promotion is None]
    return rng.choice(fallback or list(board.legal_moves))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Auto-play calibrated chess: python-chess picks quiet moves, "
                    "arm replays validated routes (no runtime planning/IK)")
    parser.add_argument("--moves", type=int, default=8,
                        help="number of plies to play (default: 8)")
    parser.add_argument("--seed", type=int, default=1,
                        help="random seed for move choice (default: 1)")
    parser.add_argument("--routes", help="editable square_routes.yaml (default: installed config)")
    parser.add_argument("--speed", type=float, default=1.5,
                        help="motion speed multiplier 0.2..5.0 (default: 1.5)")
    args = parser.parse_args()
    rclpy.init()
    node = ChessExecutor(routes_path=args.routes, speed_multiplier=args.speed)
    node.reset_pieces()
    import time as _time
    _time.sleep(0.5)  # let the visualizer receive reset before the first pick
    board = chess.Board()
    rng = random.Random(args.seed)
    print(f"auto-play {args.moves} plies, seed={args.seed}")
    print(board)
    try:
        for i in range(args.moves):
            if board.is_game_over():
                print(f"game over: {board.result()}")
                break
            move = pick_move(board, rng)
            uci = move.uci()
            print(f"[{i + 1}] {board.san(move)} ({uci})")
            try:
                node.move(uci[:2], uci[2:4])
            except (ExecutionError, RuntimeError) as exc:
                node.stop()
                print(f"ERROR: {exc}")
                break
            board.push(move)
            print(board)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
