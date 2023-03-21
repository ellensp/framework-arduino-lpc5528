/*
BSD 3-Clause License

Copyright (c) 2021-2022 WPI Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "Print.h"
#include <stdarg.h>

#define ENABLE_PRINTF

// Public Methods //////////////////////////////////////////////////////////////

/* default implementation: may be overridden */
size_t Print::write(const uint8_t *buffer, size_t size) {
  size_t n = 0;
  while (size--) {
    if (write(*buffer++)) n++;
    else break;
  }
  return n;
}

size_t Print::print(const char str[])           { return write(str); }
size_t Print::print(char c)                     { return write(c); }
size_t Print::print(unsigned char b, int base)  { return print((unsigned long) b, base); }
size_t Print::print(int n, int base)            { return print((long) n, base); }
size_t Print::print(unsigned int n, int base)   { return print((unsigned long) n, base); }
size_t Print::print(double n, int digits)       { return printFloat(n, digits); }
size_t Print::print(const Printable& x)         { return x.printTo(*this); }
size_t Print::println(void)                     { return write("\r\n"); }

size_t Print::print(long n, int base) {
  if (base == 0) return write(n);
  if (base == 10) {
    if (n < 0) {
      const int t = print('-');
      return printNumber(-n, 10) + t;
    }
  }
  return printNumber(n, base);
}

size_t Print::print(unsigned long n, int base) {
  if (base == 0) return write(n);
  return printNumber(n, base);
}

#define PRINTLN(...) do{ \
  size_t n = print(__VA_ARGS__); \
  n += println(); \
  return n; \
}while(0)

size_t Print::println(const char c[])              { PRINTLN(c); }
size_t Print::println(char c)                      { PRINTLN(c); }
size_t Print::println(unsigned char b, int base)   { PRINTLN(b, base); }
size_t Print::println(int num, int base)           { PRINTLN(num, base); }
size_t Print::println(unsigned int num, int base)  { PRINTLN(num, base); }
size_t Print::println(long num, int base)          { PRINTLN(num, base); }
size_t Print::println(unsigned long num, int base) { PRINTLN(num, base); }
size_t Print::println(double num, int digits)      { PRINTLN(num, digits); }
size_t Print::println(const Printable& x)          { PRINTLN(x); }

// Private Methods /////////////////////////////////////////////////////////////

size_t Print::printNumber(unsigned long n, uint8_t base) {
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  return write(str);
}

size_t Print::printFloat(double number, uint8_t digits) {
  size_t n = 0;

  if (isnan(number)) return print("nan");
  if (isinf(number)) return print("inf");
  if (number > 4294967040.0) return print ("ovf");  // constant determined empirically
  if (number <-4294967040.0) return print ("ovf");  // constant determined empirically

  // Handle negative numbers
  if (number < 0.0) {
    n += print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i) rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  uint32_t int_part = (uint32_t)number;
  double remainder = number - (double)int_part;
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) n += print('.');

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    const int toPrint = int(remainder);
    n += print(toPrint);
    remainder -= toPrint;
  }

  return n;
}

#ifdef ENABLE_PRINTF

  size_t Print::printf(const char *argList, ...) {
    const char *ptr;
    double floatNum_f32;
    va_list argp;
    int32_t num_s32;
    uint32_t num_u32;
    char *str;
    char  ch;
    uint8_t numOfDigits;
    size_t length = 0;

    va_start(argp, argList);

    // Loop through the list to extract all the input arguments
    for (ptr = argList; (ch = *ptr); ptr++) {
      if (ch == '%') {       //Check for '%' as there will be format specifier after it
        ptr++;
        ch = *ptr;
        if (ch >= '0' && ch <= '9') {
          numOfDigits = 0;
          while (ch >= '0' && ch <= '9') {
            numOfDigits = numOfDigits * 10 + ch - '0';
            ptr++;
            ch = *ptr;
          }
        }
        else
         numOfDigits = 0xFF;

        switch(ch) {     // Decode the type of the argument

        case 'C':
        case 'c':     // Argument type is of char, hence read char data from the argp
            ch = va_arg(argp, int);
            length += print(ch);
            break;

        case 'd':    // Argument type is of signed integer, hence read 16bit data from the argp
        case 'D':
            num_s32 = va_arg(argp, int);
            length += print(num_s32, 10);
            break;

        case 'u':
        case 'U':    // Argument type is of integer, hence read 32bit unsigend data
            num_u32 = va_arg(argp, uint32_t);
            length += print(num_u32, 10);
            break;

        case 'x':
        case 'X':  // Argument type is of hex, hence hexadecimal data from the argp
            num_u32 = va_arg(argp, uint32_t);
            length += print(num_u32, 16);
            break;

        case 'b':
        case 'B':  // Argument type is of binary,Read int and convert to binary
            num_u32 = va_arg(argp, uint32_t);
            length += print(num_u32, 2);
            break;

        case 'F':
        case 'f': // Argument type is of float, hence read double data from the argp
            floatNum_f32 = va_arg(argp, double);
            length += printFloat(floatNum_f32, 10);
            break;

        case 'S':
        case 's': // Argument type is of string, hence get the pointer to sting passed
            str = va_arg(argp, char *);
            length += print(str);
            break;

        case '%':
          length += print('%');
          break;
        }
      }
      else
        length += print(ch); // As '%' is not detected transmit the char passed
    }

    va_end(argp);
    return length;
  }

#endif // ENABLE_PRINTF
