
#undef NDEBUG

#include <easynet/shared_buffer.hpp>

#include <iostream>
#include <cassert>

/*************************************************************************************************/

#define STRINGIZE_IMPL(x) \
    #x
#define STRINGIZE(x) STRINGIZE_IMPL(x)

#define TEST_COND(l, cmp, r) \
    do { \
        assert((l) cmp (r)); \
    } while( false )

#define TEST_LESS(l, r)     TEST_COND(l, <, r)
#define TEST_LESSEQ(l, r)   TEST_COND(l, <=, r)
#define TEST_EQ(l, r)       TEST_COND(l, ==, r)
#define TEST_NEQ(l, r)      TEST_COND(l, !=, r)
#define TEST_GR(l, r)       TEST_COND(l, >, r)
#define TEST_GREQ(l, r)     TEST_COND(l, >=, r)
#define TEST_ZERO(v)        TEST_COND(v, ==, 0)
#define TEST_NOTZERO(v)     TEST_COND(v, !=, 0)
#define TEST_NULL(v)        TEST_COND(v, ==, nullptr)
#define TEST_NOTNULL(v)     TEST_COND(v, !=, nullptr)

/*************************************************************************************************/

void test_alloc() {
    {
        auto b = easynet::buffer_alloc(0);

        TEST_NULL(b.data);
        TEST_EQ(b.size, 0);
        TEST_EQ(easynet::buffer_size(b), 0);
    }

    {
        constexpr auto size = 1;
        auto b = easynet::buffer_alloc(size);

        TEST_NOTNULL(b.data);
        TEST_EQ(b.size, size);
        TEST_EQ(easynet::buffer_size(b), size);
    }
}

void test_resize() {
    constexpr auto size = 1;
    auto b = easynet::buffer_alloc(size);

    easynet::buffer_resize(b, size+1);
    TEST_NOTNULL(b.data);

    TEST_EQ(b.size, size+1);
    TEST_EQ(easynet::buffer_size(b), size+1);

    easynet::buffer_resize(b, size);
    TEST_NOTNULL(b.data);

    TEST_EQ(b.size, size);
    TEST_EQ(easynet::buffer_size(b), size);
}

void test_clone() {
    {
        constexpr auto size = 1;
        auto b = easynet::buffer_alloc(size);

        b.data.get()[0] = 'a';

        auto c = easynet::buffer_clone(b);
        TEST_NOTNULL(c.data);

        TEST_EQ(c.size, size);
        TEST_EQ(easynet::buffer_size(c), size);

        TEST_NEQ(b.data.get(), c.data.get());
        TEST_NEQ(easynet::buffer_data(b), easynet::buffer_data(c));

        TEST_EQ(easynet::buffer_data(c)[0], 'a');
    }

    {
        constexpr auto size = 2;
        auto b = easynet::buffer_alloc(size);

        b.data.get()[0] = 'a';
        b.data.get()[1] = 'b';

        auto c = easynet::buffer_clone(b, size-1);
        TEST_NOTNULL(c.data);

        TEST_EQ(c.size, size-1);
        TEST_EQ(easynet::buffer_size(c), size-1);

        TEST_NEQ(b.data.get(), c.data.get());
        TEST_NEQ(easynet::buffer_data(b), easynet::buffer_data(c));

        TEST_EQ(easynet::buffer_data(c)[0], 'a');
    }

    {
        constexpr auto size = 2;
        auto b = easynet::buffer_alloc(size);

        b.data.get()[0] = 'a';
        b.data.get()[1] = 'b';

        auto c = easynet::buffer_clone(b, 1, 1);
        TEST_NOTNULL(c.data);

        TEST_EQ(c.size, 1);
        TEST_EQ(easynet::buffer_size(c), 1);

        TEST_NEQ(b.data.get(), c.data.get());
        TEST_NEQ(easynet::buffer_data(b), easynet::buffer_data(c));

        TEST_EQ(easynet::buffer_data(c)[0], 'b');
    }
}

void test_shift() {
    constexpr auto size = 2;
    auto b = easynet::buffer_alloc(size);

    b.data.get()[0] = 'a';
    b.data.get()[1] = 'b';

    auto c = easynet::buffer_lshift(b, 1);
    TEST_NOTNULL(c.data);

    TEST_EQ(easynet::buffer_size(c), 1);
    TEST_EQ(easynet::buffer_data(c)[0], 'b');
}

/*************************************************************************************************/

int main() {
    test_alloc();
    test_resize();
    test_clone();
    test_shift();

    return 0;
}

/*************************************************************************************************/
