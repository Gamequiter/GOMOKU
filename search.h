// search.h
// Very simple move selection engine for Gomoku.

#ifndef GOMOKU_SEARCH_H
#define GOMOKU_SEARCH_H

#include "board.h"

#include <chrono>
#include <vector>
#include <unordered_map>
#include <cstddef>

#include "history_heuristic.h"

namespace gomoku {

// A basic search engine that selects a move using iterative deepening
// alpha–beta search with a transposition table, killer moves and a
// history heuristic.
class SearchEngine {
public:
    SearchEngine();

    // Find the best move for myColor within the given time limit (milliseconds).
    // The board is passed by non-const reference so that the engine can make
    // and unmake moves during the search.
    Move findBestMove(Board &board, Player myColor, int timeLimitMs);

private:
    // Opening book logic: for the predetermined starting position, optionally
    // return a simple hard-coded opening move.  Returns true and sets outMove
    // if a book move is available, otherwise returns false.
    bool getOpeningMove(const Board &board, Player myColor, Move &outMove) const;

    // Timer helpers.
    void startTimer(int timeLimitMs);
    bool timeUp() const;

    // Convert a pattern (contiguous run length and openness on both sides)
    // into a score.  Used by evaluatePlayer().
    int patternScore(int count, bool leftOpen, bool rightOpen) const;

    // Evaluate the position for a single player by scanning rows, columns
    // and diagonals for patterns of that player's stones.
    int evaluatePlayer(const Board &board, Player player) const;

    // Overall evaluation from the perspective of myColor.
    int evaluate(const Board &board, Player myColor) const;

    // Core alpha–beta search routine.
    // - depth: remaining search depth.
    // - alpha/beta: current search window.
    // - currentPlayer: which player is to move at this node.
    // - myColor: the player we are evaluating for.
    // - ply: distance from the root (used for killer moves).
    int alphaBeta(Board &board,
                  int depth,
                  int alpha,
                  int beta,
                  Player currentPlayer,
                  Player myColor,
                  int ply);

    // Generate and order moves for the current node using heuristics
    // (killer moves, history heuristic, simple proximity scoring, etc.).
    std::vector<Move> orderMoves(Board &board,
                                 Player currentPlayer,
                                 Player myColor,
                                 int ply);

    // Compute the set of defensive squares (cost squares) for the specified
    // player on the given board.  These are squares that must be filled
    // by the opponent to stop an immediate threat.  A four‑in‑a‑row with
    // at least one open end contributes its open ends; if allowThree is
    // true, a three‑in‑a‑row with two open ends also contributes its ends.
    // The returned vector contains unique squares and does not include
    // occupied cells.  This helper is used by the search engine to
    // recognise and respond to opponent threats.
    std::vector<Move> computeCostSquares(const Board &board,
                                         Player player,
                                         bool allowThree) const;

    // --- State ---

    // Maximum search depth reached in the current iterative deepening run.
    int maxDepthReached;

    // Absolute end time for the current search (steady clock).
    std::chrono::steady_clock::time_point timeEnd;

    // Transposition table entry describing a previously searched position.
    // depth   – depth the node was searched to.
    // score   – evaluation score from the perspective of myColor.
    // flag    – 0 = exact, 1 = lower bound, 2 = upper bound.
    // bestMove – best move found at that node.
    struct TTEntry {
        int depth;
        int score;
        int flag;      // 0 = exact, 1 = lower bound, 2 = upper bound
        Move bestMove;
    };

    // Transposition table keyed by the board's Zobrist hash (Board::getHashKey()).
    std::unordered_map<uint64_t, TTEntry> transTable;

    // Hard upper bound on the number of entries stored in the
    // transposition table for a single search.  This prevents
    // unbounded growth in very long searches.
    static const std::size_t TT_MAX_ENTRIES = 200000;

    // --- Killer move heuristics ---
    // Killer moves are moves that caused a beta cutoff at a given search ply.
    // For each ply in the current search tree we store up to two killer moves
    // that will be tried early in move ordering.  We reset these on each new
    // search.
    static const int MAX_PLY = 64;
    Move killerMoves[MAX_PLY][2];

    // History heuristic table.  Records how often moves cause cutoffs to
    // further improve move ordering.  It is reset at the start of each
    // search.
    HistoryHeuristic history;
};

} // namespace gomoku

#endif // GOMOKU_SEARCH_H