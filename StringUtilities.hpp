#pragma once

#include <algorithm>
#include <string>
#include <string_view>
#include <charconv>
#include <stdexcept>
#include <limits>
#include <cstring>
#include <cctype>
#include <iterator>

void Console_Print(const char* fmt, ...);

namespace pUtils {

	//Not used anymore
	inline std::string extractFirstLine(std::string_view data) {
		auto pos = data.find('\n');
		return std::string(data.data(), pos == std::string_view::npos ? data.size() : pos);
	}

	inline std::string toLowerCase(std::string_view input) {
		std::string out;
		out.reserve(input.size());
		std::transform(input.begin(), input.end(), std::back_inserter(out),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		return out;
	}

	inline std::string sanitizeTag(std::string_view input) {
		std::string out;
		out.reserve(input.size());
		for (unsigned char c : input) {
			if (!std::isspace(c)) {
				out.push_back(static_cast<char>(std::tolower(c)));
			}
		}
		return out;
	}

	inline constexpr unsigned char asciiLowerBT(unsigned char c) noexcept {
		return (c >= 'A' && c <= 'Z') ? char(c | 0x20) : c;
	}

	inline unsigned char asciiLower(unsigned char c) noexcept {
		return (c >= 'A' && c <= 'Z') ? char(c + ('a' - 'A')) : c;
	}

	inline std::unique_ptr<char[]> toLowerCase_up(const char* s) {
		if (!s) return {};
		auto lower = pUtils::toLowerCase(s);
		auto buf = std::make_unique<char[]>(lower.size() + 1);
		std::memcpy(buf.get(), lower.data(), lower.size() + 1);
		return buf;
	}

	constexpr UInt32 ToUInt32(const char* str) {
		if (!str) throw std::invalid_argument("Null string");
		UInt32 result = 0;
		for (const char* p = str; *p; ++p) {
			char c = *p;
			if (c < '0' || c > '9')
				throw std::invalid_argument("Invalid integer string");
			UInt32 digit = static_cast<UInt32>(c - '0');
			if (result > (std::numeric_limits<UInt32>::max() - digit) / 10)
				throw std::overflow_error("Integer overflow");
			result = result * 10 + digit;
		}
		return result;
	}

	// ————————————————————————————————————————————————————————————
	// Runtime version using std::from_chars (no allocations, no locales).
	inline UInt32 ToUInt32(std::string_view s) {
		UInt32 value = 0;
		auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
		if (ec == std::errc::invalid_argument)
			throw std::invalid_argument("Invalid integer string");
		if (ec == std::errc::result_out_of_range)
			throw std::overflow_error("Integer overflow");
		return value;
	}

	// ————————————————————————————————————————————————————————————
	// Try‐parse a double; returns false on bad format or out‐of‐range.
	inline bool TryParseDouble(std::string_view s, double& outVal) {
		try {
			size_t pos;
			// stod requires a std::string, so make one on the side.
			outVal = std::stod(std::string(s), &pos);
			return pos == s.size();
		}
		catch (...) {
			return false;
		}
	}

	inline uint16_t floatToHalf(float f) {
		// simple 1:1 sign‐exp‐mantissa split → half.  For demo purposes only:
		// – clamp to [–65504..+65504]
		// – no subnormals, no NaN handling, etc.
		uint32_t bits = *reinterpret_cast<uint32_t*>(&f);
		uint32_t sign = (bits >> 31) & 1;
		int32_t  exp = int32_t((bits >> 23) & 0xFF) - 127 + 15;
		uint32_t man = (bits >> 13) & 0x3FF;

		if (exp <= 0)   exp = 0, man = 0;
		if (exp >= 31)  exp = 31, man = 0;

		return uint16_t((sign << 15) | (exp << 10) | man);
	}

	// ————————————————————————————————————————————————————————————
	// True if the entire string is an integer literal (optional +/-, then all digits).
	inline bool isNumber(std::string_view s) {
		if (s.empty()) return false;
		size_t i = (s[0] == '+' || s[0] == '-') ? 1 : 0;
		if (i == 1 && s.size() == 1) return false;
		return std::all_of(s.begin() + i, s.end(),
			[](unsigned char c) { return std::isdigit(c); });
	}

	inline char* findChar(const char* s, char ch) noexcept {
		if (!s) return nullptr;
		using U8 = uint8_t;

		const U8 uc = static_cast<U8>(ch);

	#if INTPTR_MAX == INT64_MAX
		// ---------------------- 64-bit SWAR version ----------------------
		using U64 = uint64_t;
		// build 0xCC.. mask of the target byte
		U64 mask = uc;
		mask |= mask << 8;
		mask |= mask << 16;
		mask |= mask << 32;

		// align to 8-byte boundary
		while (reinterpret_cast<uintptr_t>(s) & 7) {
			if (*s == uc) return const_cast<char*>(s);
			if (*s == 0 ) return nullptr;
			++s;
		}

		auto* wp = reinterpret_cast<const U64*>(s);
		for (;;) {
			U64 v = *wp++;
			// detect match or zero in any byte
			U64 x        = v ^ mask;
			U64 hasMatch = (x - 0x0101010101010101ULL) & ~x & 0x8080808080808080ULL;
			U64 hasZero  = (v - 0x0101010101010101ULL) & ~v & 0x8080808080808080ULL;
			if (hasMatch || hasZero) {
				const U8* bp = reinterpret_cast<const U8*>(wp) - 8;
				for (int i = 0; i < 8; ++i) {
					if (bp[i] == uc) return const_cast<char*>(reinterpret_cast<const char*>(bp + i));
					if (bp[i] == 0 ) return nullptr;
				}
			}
		}

	#elif INTPTR_MAX == INT32_MAX
		// ---------------------- 32-bit SWAR version ----------------------
		using U32 = uint32_t;
		U32 mask32 = uc;
		mask32 |= mask32 << 8;
		mask32 |= mask32 << 16;
		mask32 |= mask32 << 24;

		// align to 4-byte boundary
		while (reinterpret_cast<uintptr_t>(s) & 3) {
			if (*s == uc) return const_cast<char*>(s);
			if (*s == 0 ) return nullptr;
			++s;
		}

		auto* wp32 = reinterpret_cast<const U32*>(s);
		for (;;) {
			U32 v = *wp32++;
			U32 x        = v ^ mask32;
			U32 hasMatch = (x - 0x01010101u) & ~x & 0x80808080u;
			U32 hasZero  = (v - 0x01010101u) & ~v & 0x80808080u;
			if (hasMatch || hasZero) {
				const U8* bp = reinterpret_cast<const U8*>(wp32) - 4;
				for (int i = 0; i < 4; ++i) {
					if (bp[i] == uc) return const_cast<char*>(reinterpret_cast<const char*>(bp + i));
					if (bp[i] == 0 ) return nullptr;
				}
			}
		}

	#else
		// --------------------- Fallback to C runtime ----------------------
		return const_cast<char*>(std::strchr(s, ch));
	#endif
	}

}