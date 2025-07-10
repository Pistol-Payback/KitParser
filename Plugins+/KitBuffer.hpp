#pragma once
#include <array>
#include <iostream>
#include <fstream>
#include <filesystem>

#define NOMINMAX
#include <Windows.h>

#include "KitNesting.hpp"
#include "StringUtilities.hpp"

namespace Kit {

    struct FileMapping {

        HANDLE  fileHandle = INVALID_HANDLE_VALUE;
        HANDLE  mapping = nullptr;
        LPVOID  view = nullptr;
        size_t  size = 0;

        // Open, map, and initialize everything
        bool map(const std::wstring& path) {
            // Clean up any previous mapping
            reset();

            fileHandle = CreateFileW(
                path.c_str(),
                GENERIC_READ,
                FILE_SHARE_READ,
                nullptr,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                nullptr
            );
            if (fileHandle == INVALID_HANDLE_VALUE)
                return false;

            LARGE_INTEGER fi;
            if (!GetFileSizeEx(fileHandle, &fi) || fi.QuadPart == 0)
                return false;
            size = static_cast<size_t>(fi.QuadPart);

            mapping = CreateFileMappingW(
                fileHandle, nullptr,
                PAGE_READONLY,
                0, 0,
                nullptr
            );
            if (!mapping)
                return false;

            view = MapViewOfFile(
                mapping,
                FILE_MAP_READ,
                0, 0,
                0
            );
            if (!view)
                return false;

            return true;
        }

        // Unmap & close handles
        void reset() {
            if (view) { UnmapViewOfFile(view); view = nullptr; }
            if (mapping) { CloseHandle(mapping); mapping = nullptr; }
            if (fileHandle != INVALID_HANDLE_VALUE) {
                CloseHandle(fileHandle);
                fileHandle = INVALID_HANDLE_VALUE;
            }
            size = 0;
        }

        // Convenience: raw pointer to data
        const char* data() const {
            return static_cast<const char*>(view);
        }

        // Destructor will clean up
        ~FileMapping() {
            reset();
        }
    };

    enum class Encoding : uint8_t {
        Unknown,
        Ascii,       // no BOM or UTF-8 without BOM
        Cp1251,      // single-byte Cyrillic
        Utf8Bom,     // EF BB BF
        Utf16LeBom,  // FF FE
        Utf16BeBom,  // FE FF
    };

    struct KitReadBufferAL
    {

        //Member vars...................................

            FileMapping fileMapping;

            const char* buffer = nullptr;
            size_t      size = 0;
            const char* current = nullptr;

            Encoding    encoding = Encoding::Unknown;
            std::string utf8Buffer;  // holds converted UTF-16→UTF-8

            size_t      lineCount = 1;

        //............................................

        KitReadBufferAL() = default;
        KitReadBufferAL(const KitReadBufferAL&) = delete;
        KitReadBufferAL& operator=(const KitReadBufferAL&) = delete;

        inline const char* end() const { return buffer + size; }
        inline const char* begin() const { return buffer; }
        inline size_t getBufferIndex() const { return current - buffer; }

    private:

        #define CHECK_BUFFER_BOUNDS() \
            if (current == end()) { return false; }

//-----------------------------------------------------
// Healper functions
//-----------------------------------------------------

        //Values that indicate a delimiter
        inline bool isEndLine() const {
            return current == end() || current[0] == '\n' || current[0] == '{' || current[0] == '}' || (current + 1 < end() && current[0] == '/' && current[1] == '/');
        }

        // Check if we've reached the end, or we have hit a comment block
        inline bool isCommentBlock() const {
            return current == end() || (current[0] == '/');
        }

        inline void skipPerLineWhiteSpace() {
            while (current != end() && std::isspace(static_cast<unsigned char>(current[0])) && !isEndLine()) {
                increment_Iterator();
            }
        }

        inline bool nextLine()
        {
            // Optional approach: memchr can be faster than a manual while loop:
            const char* newlinePos = static_cast<const char*>(std::memchr(current, '\n', end() - current));
            if (!newlinePos) {
                // No newline found -> skip to end
                current = end();
                return false;
            }
            // We found a newline at p
            current = newlinePos + 1; // +1 to skip the '\n'
            ++lineCount;
            return true;

        }

        //Call this only after function calls.
        inline bool jumpToValidSyntax()
        {
            while (true) {
                // Skip all whitespace
                while (current != end() && std::isspace(static_cast<unsigned char>(current[0]))) {
                    increment_Iterator();
                }

                // If it's an empty line, or comment line, skip it by calling nextLine()
                if (isCommentBlock()) {
                    if (!nextLine()) {
                        return false; //End of buffer
                    }
                }
                else {
                    return true; //Text found
                }

            }

        }

        // Simpler ASCII-only tolower for performance
        static inline constexpr unsigned char lowerASCII(unsigned char c) noexcept {
            return pUtils::asciiLowerBT(c);
        }

        //Clamps numerical types to their specified range
        template <typename T>
        constexpr T clampToType(unsigned int accumulator, bool negative, const char* restorePos)
        {
            static_assert(std::is_integral_v<T>, "T must be an integral type");
            static_assert(!std::is_same<T, bool>::value, "T must not be bool");

            if constexpr (std::is_signed_v<T>) {
                // Signed path
                // We'll convert to an int (max 32-bit) before final cast to smaller T

                // Because the absolute value of INT_MIN is 2147483648 (one more than INT_MAX).
                const unsigned int absOfIntMin = 2147483648U; // For -2147483648

                int signed32 = 0;
                if (negative) {
                    // e.g. user typed -3000000000 => overflow
                    if (accumulator > absOfIntMin) {
                        current = restorePos;
                        throw std::runtime_error("Overflow: negative value out of 32-bit range.");
                    }
                    else if (accumulator == absOfIntMin) {
                        // This is exactly "-2147483648"
                        signed32 = (std::numeric_limits<int>::min)();
                    }
                    else {
                        signed32 = -static_cast<int>(accumulator);
                    }
                }
                else {
                    // Non-negative => must fit in [0..INT_MAX]
                    if (accumulator > static_cast<unsigned int>((std::numeric_limits<int>::max)())) {
                        current = restorePos;
                        throw std::runtime_error("Overflow: value out of signed 32-bit range.");
                    }
                    signed32 = static_cast<int>(accumulator);
                }

                // Finally check if signed32 fits into smaller type T (e.g. int8_t)
                if (signed32 < (std::numeric_limits<T>::min)() ||
                    signed32 > (std::numeric_limits<T>::max)())
                {
                    current = restorePos;
                    throw std::runtime_error("Parsed integer doesn't fit target type range.");
                }

                return static_cast<T>(signed32);
            }
            else {
                // Unsigned path
                // Already guaranteed 0 <= accumulator <= UINT32_MAX
                // Just check if it fits in T (e.g. uint8_t, uint16_t, etc.)
                if (accumulator > (std::numeric_limits<T>::max)()) {
                    current = restorePos;
                    throw std::runtime_error("Parsed integer doesn't fit target unsigned type range.");
                }

                return static_cast<T>(accumulator);
            }
        }

        // getNextFunction (std::string version)
        inline bool getFirstFunction(std::string& output) {

            // Skip whitespace and comment lines, returns false if end of buffer
            if (!jumpToValidSyntax()) {
                return false; //end of buffer
            }

            // Collect characters until whitespace
            while (current != end() && !std::isspace(static_cast<unsigned char>(current[0])))
            {
                output.push_back(lowerASCII(static_cast<unsigned char>(current[0])));
                increment_Iterator();
            }

            // If empty, no argument
            if (output.empty()) {
                return false;
            }

            // Trim trailing colon
            if (output.back() == ':') {
                output.pop_back();
            }

            return true; // Returns the function name or an empty string if not found

        }

        // Detect BOM and convert entire buffer to UTF-8 if needed
        bool detectAndConvertEncoding() {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(buffer);
            // UTF-8 BOM
            if (size >= 3 && p[0] == 0xEF && p[1] == 0xBB && p[2] == 0xBF) {
                encoding = Encoding::Utf8Bom;
                buffer += 3; size -= 3;
                return true;
            }
            // UTF-16 LE BOM
            if (size >= 2 && p[0] == 0xFF && p[1] == 0xFE) {
                encoding = Encoding::Utf16LeBom;
                return convertUtf16LeToUtf8(buffer + 2, size - 2);
            }
            // UTF-16 BE BOM
            if (size >= 2 && p[0] == 0xFE && p[1] == 0xFF) {
                encoding = Encoding::Utf16BeBom;
                return convertUtf16BeToUtf8(buffer + 2, size - 2);
            }
            // No BOM: assume UTF-8 without BOM
            encoding = Encoding::Utf8Bom;
            return true;
        }

        // Convert UTF-16 LE buffer to UTF-8 in utf8Buffer
        bool convertUtf16LeToUtf8(const char* data, size_t dataSize) {
            int wideCount = static_cast<int>(dataSize / 2);
            const wchar_t* wideSrc = reinterpret_cast<const wchar_t*>(data);

            int utf8Len = WideCharToMultiByte(
                CP_UTF8, 0,
                wideSrc, wideCount,
                nullptr, 0,
                nullptr, nullptr);
            if (utf8Len <= 0)
                return false;

            utf8Buffer.resize(utf8Len);
            WideCharToMultiByte(
                CP_UTF8, 0,
                wideSrc, wideCount,
                utf8Buffer.data(), utf8Len,
                nullptr, nullptr);

            // swap in utf8Buffer
            FileMapping old = std::move(fileMapping);
            buffer = utf8Buffer.data();
            size = utf8Buffer.size();
            return true;
        }

        // Convert UTF-16 BE buffer (big-endian) to UTF-8 in utf8Buffer
        bool convertUtf16BeToUtf8(const char* data, size_t dataSize) {
            // allocate temporary wide buffer and byte-swap
            int count = static_cast<int>(dataSize / 2);
            std::vector<wchar_t> wbuf(count);
            const uint8_t* u8 = reinterpret_cast<const uint8_t*>(data);
            for (int i = 0; i < count; ++i) {
                // big-endian to little-endian
                wbuf[i] = static_cast<wchar_t>(u8[2 * i] << 8 | u8[2 * i + 1]);
            }

            int utf8Len = WideCharToMultiByte(
                CP_UTF8, 0,
                wbuf.data(), count,
                nullptr, 0,
                nullptr, nullptr);
            if (utf8Len <= 0)
                return false;

            utf8Buffer.resize(utf8Len);
            WideCharToMultiByte(
                CP_UTF8, 0,
                wbuf.data(), count,
                utf8Buffer.data(), utf8Len,
                nullptr, nullptr);

            FileMapping old = std::move(fileMapping);
            buffer = utf8Buffer.data();
            size = utf8Buffer.size();
            return true;
        }

        void resetIterator() {
            current = buffer;
            lineCount = 1;
        }

//-----------------------------------------------------
// Main logic
//-----------------------------------------------------

    public:

        // Attempts to open and prepare the file. Returns false on unsupported encoding.
        bool openFile(const std::wstring& filePath, std::string& firstFunction) {
            // map the file
            if (!fileMapping.map(filePath))
                return false;

            // initialize parser pointers
            buffer = fileMapping.data();
            size = fileMapping.size;
            current = buffer;

            // detect/convert encoding...
            if (!detectAndConvertEncoding())
                return false;

            // reset iterator & read first function
            resetIterator();
            return getFirstFunction(firstFunction);
        }

        void close() {
            fileMapping = FileMapping{};
            buffer = nullptr;
            current = nullptr;
            size = 0;
            lineCount = 1;
        }

//-----------------------------------------------------
// Word processing
//-----------------------------------------------------

        size_t getCurrentLineNumber() {
            return lineCount;
        }

        //Mainly for testing
        inline const char* increment_Iterator() {
            #ifdef _DEBUG
                // Perform bounds check in debug mode
                if (current == end()) {
                    throw std::out_of_range("Buffer iterator exceeds buffer size.");
                }
            #endif

            if (current[0] == '\n') {
                ++lineCount;
            }

            return current++;
        }

        //For error dumping
        std::string_view getCurrentLine() {

            //Save the original bufferIter
            const char* originalPos = current;

            //Go to the start of the line
            jumpBackToLineStart();
            skipPerLineWhiteSpace(); //Skip indents

            const char* startPos = current;
            while (current != end() && current[0] != '\n') {
                increment_Iterator();
            }
            size_t endPos = current - startPos;

            std::string_view line(startPos, endPos);

            //Restore bufferIter
            current = originalPos;

            return line;
        }

        void jumpTo(const char* bufferPos) {
            current = bufferPos;
        }


        //Destructive
        inline const char* jumpBackToLineStart() {
            //Go to the start of the line
            while (current >= begin() && current[0] != '\n') {
                --current;
            }
            current++; //skip \n
            return current;
        }

        //Skip document
        void jumpToBufferEnd() {
            current = buffer + size;
        }

        bool getNextArgument(float& output) {

            CHECK_BUFFER_BOUNDS();

            // Skip leading whitespace
            skipPerLineWhiteSpace();
            if (isEndLine()) return false;

            const char* startPos = current;

            // Optional leading sign
            if (current < end() && (*current == '+' || *current == '-')) {
                increment_Iterator();
            }

            bool hasDigits = false;

            // Integer part digits
            while (current < end() && std::isdigit(static_cast<unsigned char>(*current))) {
                hasDigits = true;
                increment_Iterator();
            }

            // Optional fractional part
            if (current < end() && *current == '.') {
                increment_Iterator();
                // Must have at least one digit in fraction or integer
                bool fracDigits = false;
                while (current < end() && std::isdigit(static_cast<unsigned char>(*current))) {
                    fracDigits = true;
                    increment_Iterator();
                }
                hasDigits = hasDigits || fracDigits;
            }

            if (!hasDigits) {
                // No digits at all → not a float
                current = startPos;
                return false;
            }

            // Optional exponent part
            if (current < end() && (*current == 'e' || *current == 'E')) {
                const char* expStart = current;
                increment_Iterator();

                // Exponent sign
                if (current < end() && (*current == '+' || *current == '-')) {
                    increment_Iterator();
                }

                // Must have at least one exponent digit
                bool expDigits = false;
                while (current < end() && std::isdigit(static_cast<unsigned char>(*current))) {
                    expDigits = true;
                    increment_Iterator();
                }

                if (!expDigits) {
                    // Roll back the 'e' (and optional sign) if no digits followed
                    current = expStart;
                }
            }

            // Copy substring into a small buffer
            size_t length = static_cast<size_t>(current - startPos);
            if (length >= 32) {
                throw std::runtime_error("getNextArgument float overflow");
            }
            char buf[33];
            std::memcpy(buf, startPos, length);
            buf[length] = '\0';

            // Convert with strtof
            errno = 0;
            char* endPtr = nullptr;
            float   val = std::strtof(buf, &endPtr);

            // If no characters consumed, or overflow/underflow, roll back
            if (endPtr == buf || errno == ERANGE) {
                current = startPos;
                return false;
            }

            // Success
            output = val;
            return true;
        }

        // getQuotedString: parse a quoted string in the current line. 
        bool getQuotedString(std::string_view& out)
        {
            out = {}; // reset

            // Skip leading whitespace
            skipPerLineWhiteSpace();
            char delim = current[0];
            if (delim != '\"' && delim != '`') {
                return false;
            }

            // Skip opening delimiter
            increment_Iterator();
            const char* startPos = current;

            while (current != end() && current[0] != '\n') {
                if (current[0] == delim) {
                    // found closing delimiter
                    out = std::string_view(startPos, current - startPos);
                    increment_Iterator(); // eat closing delimiter
                    return true;
                }
                increment_Iterator();
            }

            // no closing delimiter found — rewind
            current = startPos;
            return false;
        }

        // MultiLangiage
        bool getQuotedStringMultiLanguage(std::string& out) {

            std::string_view tmp;
            if (!getQuotedString(tmp)) return false;

            // 1) UTF-8 → wide
            int wlen = MultiByteToWideChar(
                CP_UTF8, 0,
                tmp.data(), int(tmp.size()),
                nullptr, 0
            );
            if (wlen <= 0) return false;
            std::wstring wbuf(wlen, L'\0');
            MultiByteToWideChar(
                CP_UTF8, 0,
                tmp.data(), int(tmp.size()),
                wbuf.data(), wlen
            );

            // 2) wide → CP-1251
            int clen = WideCharToMultiByte(
                1251, 0,
                wbuf.data(), wlen,
                nullptr, 0,
                nullptr, nullptr
            );
            if (clen <= 0) return false;
            out.resize(clen);
            WideCharToMultiByte(
                1251, 0,
                wbuf.data(), wlen,
                out.data(), clen,
                nullptr, nullptr
            );

            return true;
        }

        bool getNextArgument(char& out)
        {
            // Skip leading whitespace
            skipPerLineWhiteSpace();
            if (isEndLine()) {
                return false;
            }
            out = current[0];
            skipArgument();
            return true;
        }

        template <typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
        bool getNextArgument(T& output)
        {

            CHECK_BUFFER_BOUNDS();

            this->skipPerLineWhiteSpace();
            if (isEndLine())
                return false;

            // Save position in case we need to restore
            const char* restorePos = current;

            // Optional sign
            bool negative = false;
            if constexpr (std::is_signed_v<T>) {
                if (current != end() && current[0] == '-') {
                    negative = true;
                    increment_Iterator();
                }
            }
            else {
                if (current != end() && current[0] == '-') {
                    current = restorePos;
                    throw std::runtime_error("Encountered negative sign for an unsigned type.");
                }
            }

            // Accumulate digits into a 32-bit unsigned
            unsigned accumulator = 0;
            while (current != end() && std::isdigit(static_cast<unsigned char>(current[0]))) {
                unsigned digit = static_cast<unsigned>(current[0] - '0');
                // Overflow check for 32-bit
                if (accumulator > ((std::numeric_limits<unsigned>::max)() - digit) / 10U) {
                    current = restorePos;
                    throw std::runtime_error("Integer overflow (> 32 bits).");
                }
                accumulator = accumulator * 10U + digit;
                increment_Iterator();
            }

            // Ensure we ended on whitespace or buffer end
            if (current != end() && !std::isspace(static_cast<unsigned char>(current[0]))) {
                // Non-whitespace => invalid parse
                current = restorePos;
                return false;
            }

            // 5. Convert accumulator into final T
            output = clampToType<T>(accumulator, negative, restorePos);
            return true;
        }

        template <>
        bool getNextArgument<bool>(bool& output)
        {
            CHECK_BUFFER_BOUNDS();

            this->skipPerLineWhiteSpace();
            if (isEndLine())
                return false;

            // Save position in case we need to restore it
            const char* restorePos = current;

            // If the next character is a digit, support "0" or "1"
            if (std::isdigit(static_cast<unsigned char>(current[0])))
            {
                if (current[0] == '0')
                {
                    output = false;
                    increment_Iterator();
                }
                else if (current[0] == '1')
                {
                    output = true;
                    increment_Iterator();
                }
                else
                {
                    // If it's a digit but not 0 or 1, that's invalid.
                    current = restorePos;
                    return false;
                }
            }
            // If the next character is alphabetic, expect "true" or "false"
            else if (std::isalpha(static_cast<unsigned char>(current[0])))
            {
                const char* start = current;
                // Consume alphabetic characters
                while (current != end() && std::isalpha(static_cast<unsigned char>(*current)))
                    increment_Iterator();

                size_t tokenLength = current - start;
                std::string token(start, tokenLength);

                // Convert token to lower-case for case-insensitive comparison
                for (auto& ch : token)
                    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

                if (token == "true")
                    output = true;
                else if (token == "false")
                    output = false;
                else
                {
                    // Unrecognized literal
                    current = restorePos;
                    return false;
                }
            }
            else
            {
                // Unexpected character encountered
                current = restorePos;
                return false;
            }

            // After parsing the boolean, ensure the next character is whitespace or end of buffer.
            if (current != end() && !std::isspace(static_cast<unsigned char>(*current)))
            {
                current = restorePos;
                return false;
            }

            return true;
        }

        //get view from begining to end with optional start, and end offset.
        inline std::string_view getDocumentView(size_t _start = 0)
        {
            return std::string_view(begin() + _start, size);
        }

        //get view from begining to end with optional start, and end offset.
        inline std::string_view getDocumentView(size_t _start, size_t _end)
        {
            if (_end > size) {
                throw std::out_of_range("getView was called on the kit buffer with an end size greater than the buffer size");
            }
            return std::string_view(begin() + _start, _end);
        }

        //Get view of remaining buffer
        inline std::string_view getView()
        {
            return std::string_view(current, end() - current);
        }

        //Get view of buffer starting from a passed postion in the buffer.
        inline std::string_view getView(const char* _start)
        {
            // Ensure _start is within the buffer bounds (data is the start of the buffer)
            if (_start < begin() || _start >= end()) {
                // Out-of-bounds: return an empty view.
                return std::string_view();
            }

            // Calculate the remaining length from _start to the end of the buffer.
            size_t remaining = static_cast<size_t>((begin() + size) - _start);
            return std::string_view(_start, remaining);
        }

        //Resets the passed std::string_view upon passing
        bool getNextArgument(std::string_view& output)
        {

            CHECK_BUFFER_BOUNDS();

            // Reset the output view
            output = std::string_view();

            skipPerLineWhiteSpace();
            if (isEndLine()) {
                return false;
            }

            // 3. Mark the start of the next argument
            const char* start = current;

            // 4. Advance until we hit whitespace or end of data
            while (current != end() && !std::isspace(static_cast<unsigned char>(current[0]))) {
                increment_Iterator();
            }

            // 5. Construct a string_view from [start, buffer[bufferIter]) if we haven't reached size
            size_t length = current - start;
            if (length == 0) {
                // No argument found (possible double whitespace?), return false
                return false;
            }

            // Assign the substring to the output string_view
            output = std::string_view(start, length);

            return true;
        }

        //No bounds check, but shouldn't be an issue.
        bool skipArgument()
        {
            while (!isEndLine() && !std::isspace(static_cast<unsigned char>(current[0]))) {
                increment_Iterator();
            }
            return false;
        }

        //Doesn't move the buffer pointer.
        bool peakIsNested()
        {
            auto restorePos = current;
            while (!isEndLine()) {
                increment_Iterator();
            }

            if (current[0] == '\n') {
                //Check if next char after new line is an opening brace
                while (current != end() && std::isspace(static_cast<unsigned char>(current[0]))) {
                    increment_Iterator();
                }
            }

            current = restorePos;

            if (current[0] == '{') {
                return true; //Next lines are nested
            }

            return false;
        }

        //Minimal reprocessing, Move the buffer pointer.
        bool getIsNested()
        {
            while (!isEndLine()) {
                increment_Iterator();
            }

            auto restorePos = current;

            if (current[0] == '\n') {
                //Check if next char after new line is an opening brace
                while (current != end() && std::isspace(static_cast<unsigned char>(current[0]))) {
                    increment_Iterator();
                }
            }

            if (current[0] == '{') {
                return true; //Next lines are nested
            }

            current = restorePos;

            return false;
        }

        // Call getNextFunction after this returns true.
        template<typename STATE, size_t MaxNestedLevels>
        std::optional<LockScopeGuard<NestedHandler<STATE, MaxNestedLevels>>> tryEnterLockedScope(NestedHandler<STATE, MaxNestedLevels>* nested, bool prepLockForNextState = true) {

            while (!isEndLine()) {
                increment_Iterator();
            }

            auto restorePos = current;

            if (current[0] == '\n') {
                //Check if next char after new line is an opening brace
                while (current != end() && std::isspace(static_cast<unsigned char>(current[0]))) {
                    increment_Iterator();
                }
            }

            if (current[0] == '{') {
                if (prepLockForNextState) { //Prep lock for next state. Requires that getNextFunction is called after.
                    return nested->lockNextScope();
                }

                //Enter state immediately 
                nested->EnterNestedState();
                increment_Iterator(); //eat brace
                return nested->lockCurrentScope();

            }

            // no brace → restore and return empty
            current = restorePos;
            return std::nullopt;
        }

        template<typename STATE, size_t MaxNestedLevels>
        inline bool getNextFunctionRaw(const char*& outStart, size_t& outLen, NestedHandler<STATE, MaxNestedLevels>* nested) {

            CHECK_BUFFER_BOUNDS();

            while (!isEndLine()) {
                increment_Iterator();
            }

            CHECK_BUFFER_BOUNDS(); //Check bounds before incrementing here

            // handle '{' / '}'
            if (nested) {
                char c = current[0];
                if (c == '{') {
                    nested->EnterNestedState();
                }
                else if (c == '}') {
                    increment_Iterator(); // eat brace
                    if (nested->ExitNestedState()) {
                        return false; // locked-scope breach
                    }
                }
            }

            CHECK_BUFFER_BOUNDS(); //Check bounds before incrementing here

            // consume the delimiter
            increment_Iterator();

            // skip whitespace // comments
            if (!jumpToValidSyntax()) {
                return false;
            }

            // immediate '}' handling (and recursion)
            if (nested && current[0] == '}') {
                increment_Iterator();
                if (nested->ExitNestedState()) {
                    return false;
                }
                // tail-call into ourselves
                return getNextFunctionRaw(outStart, outLen, nested);
            }

            // extract the token
            outStart = current;
            while (current != end() && !std::isspace(static_cast<unsigned char>(*current))) {
                ++current;
            }

            outLen = current - outStart;
            if (outLen == 0) {
                return false;
            }
            if (outStart[outLen - 1] == ':') {
                --outLen;
            }
            return true;
        }

        bool peekNextFunction(std::string_view& output) {

            CHECK_BUFFER_BOUNDS();
            const char* restorePoint = current;

            while (!isEndLine()) {
                increment_Iterator();
            }

            // consume the delimiter
            increment_Iterator();

            // skip whitespace/comments
            if (!jumpToValidSyntax()) {
                current = restorePoint;
                return false;
            }

            // extract the token bounds
            const char* tokenStart = current;
            while (current != end() && !std::isspace(static_cast<unsigned char>(*current))) {
                ++current;
            }
            size_t tokenLen = static_cast<size_t>(current - tokenStart);

            // zero-length or just a colon? bail out
            if (tokenLen == 0) {
                current = restorePoint;
                return false;
            }

            if (tokenStart[tokenLen - 1] == ':') {
                --tokenLen;
                if (tokenLen == 0) {
                    current = restorePoint;
                    return false;
                }
            }

            // 6) restore the parser position and return the view
            current = restorePoint;
            output = std::string_view(tokenStart, tokenLen);
            return true;
        }

        //Can be useful when kit functions want to have custom nesting logic.
        template<typename STATE, size_t MaxNestedLevels>
        bool getNextFunction(std::string_view& output, NestedHandler<STATE, MaxNestedLevels>* nested) {

            const char* start;
            size_t      len;
            if (!getNextFunctionRaw(start, len, nested)) {
                return false;
            }
            output = std::string_view(start, len);
            return true;

        }


        // string version, returns as lower case
        template<typename STATE, size_t MaxNestedLevels>
        bool getNextFunction(std::string& output, NestedHandler<STATE, MaxNestedLevels>* nested) {

            output.clear();

            const char* start;
            size_t      len;
            if (!getNextFunctionRaw(start, len, nested)) {
                return false;
            }

            // copy + lowercase
            output.assign(start, len);
            std::transform(
                output.begin(), output.end(), output.begin(),
                [](unsigned char c) { return lowerASCII(c); }
            );
            return true;

        }

        inline void skipFunction()
        {
            if (getIsNested()) { //This will script a non-nested function
                skipNestedFunction();
            }
        }

        inline void skipNestedFunction()
        {

            size_t remaining = static_cast<size_t>(end() - current);
            auto raw = std::memchr(current, '{', remaining);
            if (!raw) {
                return;
            }

            current = static_cast<const char*>(raw) + 1;
            int depth = 1;

            const char* const bufEnd = end();
            while (current < bufEnd && depth > 0) {
                char c = *current++;
                switch (c) {
                case '{':
                    ++depth;
                    break;
                case '}':
                    --depth;
                    break;
                case '"':  // skip string literals (handles escapes)
                {
                    bool esc = false;
                    while (current < bufEnd) {
                        char d = *current++;
                        if (esc) { esc = false; continue; }
                        if (d == '\\') { esc = true; }
                        else if (d == '"') { break; }
                    }
                }
                break;
                default:
                    break;
                }
            }

            return;
        }

    };

}