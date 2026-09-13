"""Auto-play chess without manual control: python-chess generates the moves.

White plies run the deterministic arm pipeline (HOME -> PICK -> HOME ->
DROP -> HOME with gripper steps). Black plies are opponent moves: no arm
motion, the board display just follows the python-chess game state.
Nothing is hard-coded; every move comes from python-chess legal move
generation. Only quiet moves (no capture, no promotion, no castling) are
chosen so no discard handling is needed.
"""
from __future__ import annotations

import argparse
import itertools
import random
import time

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
    parser.add_argument("--moves", type=int, default=0,
                        help="number of plies to play (default: 0 = full game until game over)")
    parser.add_argument("--seed", type=int, default=42,
                        help="random seed for move choice (default: 42)")
    parser.add_argument("--routes", help="editable square_routes.yaml (default: installed config)")
    parser.add_argument("--speed", type=float, default=2.0,
                        help="motion speed multiplier 0.2..5.0 (default: 2.0)")
    parser.add_argument("--move-both", action="store_true",
                        help="arm physically executes black moves too "
                             "(default: arm moves white only, black moves itself)")
    args = parser.parse_args()
    rclpy.init()
    node = ChessExecutor(routes_path=args.routes, speed_multiplier=args.speed)
    node.reset_pieces()
    time.sleep(0.5)  # let the visualizer receive reset before the first pick
    board = chess.Board()
    rng = random.Random(args.seed)
    full_game = args.moves <= 0
    print(f"auto-play {'full game' if full_game else args.moves} plies, seed={args.seed}")
    print(board)
    try:
        for i in (itertools.count() if full_game else range(args.moves)):
            if board.is_game_over():
                print(f"game over: {board.result()}")
                break
            move = pick_move(board, rng)
            uci = move.uci()
            side = "white" if board.turn == chess.WHITE else "black"
            print(f"[{i + 1}] {side}: {board.san(move)} ({uci})")
            if board.turn == chess.BLACK and not args.move_both:
                board.push(move)
                node.announce_move(uci[:2], uci[2:4])
                time.sleep(1.0)  # let the display update read as a turn
                print(board)
                continue
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
