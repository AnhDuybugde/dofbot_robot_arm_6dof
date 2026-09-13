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

from .chess_executor import ChessExecutor, config_path, load_yaml
from .trajectory_executor import ExecutionError

# Pieces as tall as the validator's worst-case envelope (king 47mm,
# queen 41mm). A NEEDS_SPECIAL_STRATEGY target is only risky while a tall
# piece still stands on a neighbouring square.
TALL_TYPES = {chess.QUEEN, chess.KING}
FILES = "abcdefgh"


def neighbour_names(square: str) -> list[str]:
    file_index, rank = FILES.index(square[0]), int(square[1]) - 1
    out = []
    for dfile in (-1, 0, 1):
        for drank in (-1, 0, 1):
            if dfile == 0 and drank == 0:
                continue
            nfile, nrank = file_index + dfile, rank + drank
            if 0 <= nfile < 8 and 0 <= nrank < 8:
                out.append(f"{FILES[nfile]}{nrank + 1}")
    return out


def load_unsafe() -> set[str]:
    try:
        data = load_yaml(config_path("piece_reachability.yaml"))
    except Exception as exc:
        print(f"guard: no reachability file ({exc}); guard disabled")
        return set()
    return {sq for sq, info in (data.get("unsafe_squares") or {}).items()
            if info.get("status") == "NEEDS_SPECIAL_STRATEGY"}


def risky(board: chess.Board, move: chess.Move, unsafe: set[str]) -> bool:
    """True when the arm should avoid this move: unsafe target square with
    a tall piece still neighbouring it after the source is vacated
    (pick happens before the carry/drop, so the source counts as empty)."""
    target = chess.square_name(move.to_square)
    if target not in unsafe:
        return False
    occupied = set(board.piece_map()) - {move.from_square}
    for name in neighbour_names(target):
        square = chess.parse_square(name)
        piece = board.piece_at(square) if square in occupied else None
        if piece is not None and piece.piece_type in TALL_TYPES:
            return True
    return False


def pick_move(board: chess.Board, rng: random.Random, unsafe: set[str],
              stats: dict) -> chess.Move:
    quiet = [m for m in board.legal_moves
             if not board.is_capture(m) and m.promotion is None
             and not board.is_castling(m)]
    if quiet:
        candidates = quiet
    else:
        # Late game may have only captures/promotions left; allow anything
        # except promotions (route replay cannot change piece shape).
        fallback = [m for m in board.legal_moves if m.promotion is None]
        candidates = fallback or list(board.legal_moves)
    safe = [m for m in candidates if not risky(board, m, unsafe)]
    if safe:
        stats["skipped"] += len(candidates) - len(safe)
        return rng.choice(safe)
    stats["forced"] += 1
    print("guard: every candidate risky, playing one anyway (game must continue)")
    return rng.choice(candidates)


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
    unsafe = load_unsafe()
    print(f"guard: {len(unsafe)} NEEDS_SPECIAL_STRATEGY squares loaded")
    stats = {"skipped": 0, "forced": 0, "arm_moves": 0}
    full_game = args.moves <= 0
    print(f"auto-play {'full game' if full_game else args.moves} plies, seed={args.seed}")
    print(board)
    try:
        for i in (itertools.count() if full_game else range(args.moves)):
            if board.is_game_over():
                print(f"game over: {board.result()}")
                break
            move = pick_move(board, rng, unsafe, stats)
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
                stats["arm_moves"] += 1
            except (ExecutionError, RuntimeError) as exc:
                node.stop()
                print(f"ERROR: {exc}")
                break
            board.push(move)
            print(board)
    finally:
        print(f"guard stats: arm_moves={stats['arm_moves']} "
              f"risky_skipped={stats['skipped']} forced_risky={stats['forced']}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
