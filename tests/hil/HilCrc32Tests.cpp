#include <gtest/gtest.h>
#include <hil/HilCrc32.h>

#include <array>
#include <cstdint>
#include <span>

using namespace aeroforge::hil;

TEST(HilCrc32Tests, EmptyInput)
{
    const std::array<std::uint8_t, 0> bytes{};

    EXPECT_EQ(calculateCrc32(bytes), 0u);
}

TEST(HilCrc32Tests, StandardReferenceVector)
{
    const std::array<std::uint8_t, 9> bytes{
        '1', '2', '3', '4', '5',
        '6', '7', '8', '9'
    };

    EXPECT_EQ(calculateCrc32(bytes), 0xCBF43926u);
}

TEST(HilCrc32Tests, DeterministicCalculation)
{
    const std::array<std::uint8_t, 7> bytes{
        0x00, 0x12, 0x34, 0xFF, 0x00, 0xAB, 0xCD
    };

    const auto original = bytes;

    const auto first = calculateCrc32(bytes);
    const auto second = calculateCrc32(bytes);

    EXPECT_EQ(first, second);
    EXPECT_EQ(bytes, original);
}

TEST(HilCrc32Tests, DetectsSingleByteModification)
{
    std::array<std::uint8_t, 8> bytes{
        1, 2, 3, 4, 5, 6, 7, 8
    };

    const auto originalCrc = calculateCrc32(bytes);

    bytes[3] ^= 0x01;

    EXPECT_NE(calculateCrc32(bytes), originalCrc);
}

TEST(HilCrc32Tests, CoversOnlySuppliedSpan)
{
    const std::array<std::uint8_t, 5> bytes{
        10, 20, 30, 40, 50
    };

    const auto prefix = std::span<const std::uint8_t>(
        bytes.data(), 4);

    const auto whole = std::span<const std::uint8_t>(bytes);

    EXPECT_NE(
        calculateCrc32(prefix),
        calculateCrc32(whole));
}