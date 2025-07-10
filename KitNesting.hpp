#pragma once

#ifdef NDEBUG
#define DEBUG_ASSERT(cond, msg) ((void)0)
#else
#include <cassert>
#define DEBUG_ASSERT(cond, msg) assert((cond) && msg)
#endif

#include <cstdint>  // For uint8_t
#include <cstddef>  // For size_t

namespace Kit {

    class MismatchedBracesException : public std::runtime_error {
    public:
        explicit MismatchedBracesException(const std::string& msg)
            : std::runtime_error(msg) {}
    };

    template <typename Handler>
    class LockScopeGuard {
    public:

        LockScopeGuard(Handler& h, UInt16 depth) noexcept
            : handler(h), lock(h.lockScope(depth))
        {}

        ~LockScopeGuard() {
            handler.unlockScope(lock);
        }

        // non‐copyable
        LockScopeGuard(const LockScopeGuard&) = delete;
        LockScopeGuard& operator=(const LockScopeGuard&) = delete;

        // movable if you really need it
        LockScopeGuard(LockScopeGuard&& o) noexcept
            : handler(o.handler), lock(o.lock)
        {
            o.lock = 0;
        }
        LockScopeGuard& operator=(LockScopeGuard&& o) noexcept {
            if (this != &o) {
                handler.unlockScope(lock);
                handler = o.handler;
                lock = o.lock;
                o.lock = 0;
            }
            return *this;
        }

        // allow manual release if you ever need to cancel the unlock/exit
        UInt16 release() noexcept {
            UInt16 t = lock;
            lock = 0;
            return t;
        }

    private:
        UInt16 lock = 0;
        Handler& handler;
    };

    //--------------------------------------------------------------------------
    // NestedHandler: Packs currentDepth & lockScopeAt into single byte (4 bits each).
    // MaxNestedLevels <= 14
    //--------------------------------------------------------------------------
    template <typename STATE, size_t MaxNestedLevels = 16>
    struct NestedHandler
    {
        static_assert(MaxNestedLevels <= 16, "MaxNestedLevels must be <= 16 to fit in an 8-bit mask.");
        static_assert(std::is_trivially_copyable_v<STATE>, "STATE must be trivially copyable for fast zeroing.");

        // Packed byte: Lower 4 bits = Depth, Upper 4 bits = LockScopeAt
        UInt16 lockMask = 0;
        UInt16 currentDepth = 0;

        // STATE storage (aligned for better cache locality)
        alignas(32) STATE nestedStates[MaxNestedLevels];

        //------------------------------------------------------------------------
        // Accessors for the nibble-based fields
        //------------------------------------------------------------------------

        __forceinline UInt16 getCurrentDepth() const noexcept {
            //return packedData & 0x0F;
            return currentDepth;
        }

        __forceinline void setCurrentDepth(UInt16 depth) noexcept {
           // packedData = (packedData & 0xF0) | (depth & 0x0F);
            currentDepth = depth;
        }

        //────────────── Lock/Unlock ──────────────
        /// Lock the current scope; returns token = 1<<depth
        __forceinline UInt16 lockScope(UInt16 depth) noexcept {
            if (depth >= MaxNestedLevels) {
                screamMessage("Exceeded max nested levels on lock");
                return 0;
            }
            UInt16 tok = UInt16(1) << (depth - 1);
            lockMask |= tok;
            return tok;
        }

        __forceinline LockScopeGuard<NestedHandler> lockCurrentScope() noexcept {
            return LockScopeGuard<NestedHandler>(*this, getCurrentDepth());
        }

        __forceinline LockScopeGuard<NestedHandler> lockNextScope() noexcept {
            return LockScopeGuard<NestedHandler>(*this, getCurrentDepth() + 1);
        }

        /// Unlock the previously locked scope
        __forceinline void unlockScope(UInt16 token) noexcept {
            lockMask &= UInt16(~token);
        }

        __forceinline bool isDepthLocked(UInt16 depth) const noexcept {
            if (depth == 0) return true;
            return (lockMask & (UInt16(1) << (depth - 1))) != 0;
        }

        //------------------------------------------------------------------------
        // Core Logic
        //------------------------------------------------------------------------

        // Enter a new nested state
        __forceinline void EnterNestedState() noexcept {

            UInt16 depth = getCurrentDepth();
            if (depth >= MaxNestedLevels) {
                screamMessage("Kit parsing error: exceeded maximum scope depth, stack overflow!");
                return;
            }

            // Copy parent state (if not root)
            if (depth) {
                nestedStates[depth] = nestedStates[depth - 1];
            }

            // Increment depth
            ++depth;

            setCurrentDepth(depth);
 
        }

        // if the depth is locked, return true, and exit from nested state
        __forceinline bool ExitNestedState() noexcept {

            UInt16 depth = getCurrentDepth();

            if (depth == 0) {
                screamMessage("Kit parsing error: tried to exit root state!");
                return false;
            }
/*
#if _DEBUG
            if (isDepthLocked(depth)) {
                    Console_Print(("DEBUG: NestedHandler lock breached on ExitNestedState() at depth " + std::to_string(depth)).c_str());
            }
#endif
*/
            // Fast zeroing of state using `memset`
            std::memset(&nestedStates[depth - 1], 0, sizeof(STATE));

            --depth;

            setCurrentDepth(depth);
            return isDepthLocked(depth + 1);

        }

        // Reset all nested states
        __forceinline void ClearNestedStates() noexcept {
            setCurrentDepth(0);
            std::memset(nestedStates, 0, sizeof(nestedStates)); // Zero out all states
            lockMask = 0;
        }

        // Return reference to current state
        __forceinline const STATE& getState() const noexcept {
            UInt16 depth = getCurrentDepth();
            if (depth == 0) {
                screamMessage("State out of scope");
                return nestedStates[depth];
            }
            return nestedStates[depth - 1];
        }

        __forceinline STATE& getState() noexcept {
            UInt16 depth = getCurrentDepth();
            if (depth == 0) {
                screamMessage("State out of scope");
            }
            return nestedStates[depth - 1];
        }

        __forceinline STATE& getNullState() {
            UInt16 depth = getCurrentDepth();
            return nestedStates[depth];
        }

        __forceinline const STATE& getNullState() const {
            UInt16 depth = getCurrentDepth();
            return nestedStates[depth];
        }

        // isEmpty => depth=0
        __forceinline bool isEmpty() const { return (getCurrentDepth() == 0); }
    };

    //-------------------------------------------------------
    // Example: specialized usage with AllocatorState
    //-------------------------------------------------------

    using AllocatorNestedHandler = NestedHandler<AllocatorState, 16>; //Increased stack size
    //using LinkerNestedHandler = NestedHandler<LinkerState, 6>;

}