# Enforcing correct operations

Consider the problem of doing calculations with units. Let's say we have some quantities
```cpp
double time = 1.5; // seconds
double distance = 10; // metres
```
Only some operations make sense. For example,
```cpp
double speed = distance / time; // m/s
```
makes sense, while the following does not
```cpp
double speed = distance + time; // oops
```

Unfortunately, the language does not enforce this. The compiler only knows that they are all `doubles`, and enforcing correctness is left to the humans (the worst place to leave it!)
But what if we could get the compiler to check the units for us? We can, and with no runtime overhead.

Scott Meyers' notes on [Dimensional Analysis in C++](https://pdfs.semanticscholar.org/f344/a75cf1ce5897d42f60811504732bce6995c7.pdf) introduce how this works for dimensional analysis.

### How far to go?

Note there are two possibilities:
- Just check for correctness
- Check for correctness and automatically perform allowed conversions

The [Boost.Units](http://www.boost.org/doc/libs/1_63_0/doc/html/boost_units.html) library is an example of the second. I will consider only the first option, which is simpler and safer.

## Enforcing reference frame correctness
Can we do the same thing for reference frames? Yes.


