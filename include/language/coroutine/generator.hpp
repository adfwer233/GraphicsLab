#pragma once

#include <coroutine>
#include <exception>
#include <iostream>
#include <iterator>
#include <utility>

namespace GraphicsLab {
template <typename Ty> struct Generator {
    struct promise_type {
        Generator get_return_object() {
            return Generator{*this};
        }
        auto initial_suspend() noexcept {
            return std::suspend_always{};
        }
        auto final_suspend() noexcept {
            return std::suspend_always{};
        }

        void return_void() noexcept {
        }

        std::suspend_always yield_value(const Ty &value) noexcept {
            current_value_ = std::addressof(value);
            return {};
        }

        void unhandled_exception() noexcept {
            exception_ = std::current_exception();
        }

        const Ty *current_value_;
        std::exception_ptr exception_;
    };

    struct iterator {
        using iterator_category = std::input_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Ty;
        using reference = const Ty &;
        using pointer = const Ty *;

        std::coroutine_handle<promise_type> coro_ = nullptr;

        iterator() = default;
        explicit iterator(std::coroutine_handle<promise_type> coro) noexcept : coro_(coro) {
        }

        iterator &operator++() {
            coro_.resume();
            if (coro_.done()) {
                coro_ = nullptr;
            }
            return *this;
        }

        void operator++(int) {
            (*this)++;
        }

        bool operator==(const iterator &right) const noexcept {
            return coro_ == right.coro_;
        }

        bool operator!=(const iterator &right) const noexcept {
            return !(*this == right);
        }

        reference operator*() const noexcept {
            return *coro_.promise().current_value_;
        }

        pointer operator->() const noexcept {
            return coro_.promise().current_value_;
        }
    };

    iterator begin() {
        if (coro_handle_) {
            coro_handle_.resume();
            if (coro_handle_.done()) {
                return {};
            }
        }

        return iterator{coro_handle_};
    }

    iterator end() noexcept {
        return {};
    }

    using handle = std::coroutine_handle<promise_type>;

    explicit Generator(promise_type &prom) noexcept
        : coro_handle_(std::coroutine_handle<promise_type>::from_promise(prom)) {
    }

    Generator(Generator &&rhs) noexcept : coro_handle_(std::exchange(rhs.coro_handle_, nullptr)) {
    }

    Generator &operator=(Generator &&right) noexcept {
        coro_handle_ = std::exchange(right.coro_handle_, nullptr);
        return *this;
    }

    ~Generator() {
        if (coro_handle_)
            coro_handle_.destroy();
    }

private:
    explicit Generator(handle h) : coro_handle_(h) {
    }
    handle coro_handle_ = nullptr;
};
}