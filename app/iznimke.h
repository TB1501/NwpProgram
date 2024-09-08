#pragma once
#include <iostream>
#include <exception>

class InvalidBeamLength : public std::exception {
public:
    const char* what() const noexcept override {
        return "Beam length cannot be zero or negative!";
    }
};

class InvalidForcePosition : public std::exception {
public:
    const char* what() const noexcept override {
        return "Position of force or moment cannot be outside the beam length!";
    }
};

class NegativeValueException : public std::exception {
public:
    const char* what() const noexcept override {
        return "Negative values are not allowed for positions or beam length!";
    }
};