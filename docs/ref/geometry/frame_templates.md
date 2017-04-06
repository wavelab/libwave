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

For an introduction to how this works, see Scott Meyers' notes on [Dimensional Analysis in C++](https://pdfs.semanticscholar.org/f344/a75cf1ce5897d42f60811504732bce6995c7.pdf).

### How far to go?

Note there are two possibilities:
- Just check for correctness
- Check for correctness and automatically perform allowed conversions

The [Boost.Units](http://www.boost.org/doc/libs/1_63_0/doc/html/boost_units.html) library is an example of the second. I will consider only the first option, which is simpler and safer.

## Enforcing reference frame correctness
Can we do the same thing for reference frames? Yes.

In kinematics, each vector quantity typically [needs three elements of decoration](http://paulfurgale.info/news/2014/6/9/representing-robot-pose-the-good-the-bad-and-the-ugly) to fully specify its meaning. Transformations between frames need two. As with units, these designators restrict which operations are allowed.

Let's start by defining some classes to serve as our Vector and Rotation types. Because this is a simplified example,
let's give the Vector just one designator, which reference frame it's expressed in. The Rotation still has two designators.

```cpp
#include <Eigen/Eigen>
using Eigen::Vector3d;
using Eigen::Matrix3d;

// The vector class, a simple wrapper of an Eigen object
// To keep this example simple, this class just has one designator: the
// reference frame it's in
template<typename Frame>
class FramedVector {
 public:
    explicit FramedVector(const Vector3d &v) : value_(v) {}
    Vector3d value() const { return value_; }

 private:
    Vector3d value_;
};

// The transformation class. Consider a rotation for now.
// It has two designators.
template<typename To, typename From>
class FramedRotation {
 public:
    explicit FramedRotation(const Matrix3d &v) : value_(v) {}
    Matrix3d value() const { return value_; }

 private:
    Matrix3d value_;
};
```

### Defining the rotation

A `FramedRotation` transforms a `FramedVector` from its `From` frame to its `To` frame. We have a rule that 
this operation is _only_ allowed if the `Frame` of the vector matches the `From` frame of the Rotation.

Here's where it gets interesting. Let's implement the rotation as the multiplication operator.

```cpp
template<typename To, typename From>
const FramedVector<To> operator*(const FramedRotation<To, From> &R,
                                 const FramedVector<From> &v) {
    return FramedVector<To>{R.value() * v.value()};
};
```

That's all it takes. We have an operation that is _only_ defined for operands that match
```cpp
FramedRotation<A, B> * FramedVector<B>
```
Anything else won't compile.

### Usage example

```cpp
#include <iostream>
int main() {
    // Define some frames (for now, they are just types)
    struct A;
    struct B;
    struct C;

    // Define an arbitrary rotation matrix to wrap in our class
    auto matrix = Matrix3d{Eigen::AngleAxisd{0.5 * M_PI, Vector3d::UnitZ()}};

    // The rotation transforms vectors from frame A into frame B
    auto R = FramedRotation<B, A>{matrix};

    // Define a vector in frame A
    auto vec_a = FramedVector<A>{Vector3d{10, 20, 30}};

    // Do the rotation
    auto result = R * vec_a;

    std::cout << "result: " << result.value().format(CommaFormat) << std::endl;
    
    // Prove that the type of the result is FramedVector<B>
    static_assert(std::is_same<FramedVector<B>, decltype(result)>::value, "!");
}
```

The following won't compile:
```cpp
    auto vec_c = FramedVector<C>{Vector3d{10, 20, 30}};
    auto not_allowed = R * vec_c; // error
```

### But how are the error messages?
For the above incorrect example, gcc spits out
```
main.cpp: In function ‘int main()’:
main.cpp:81:26: error: no match for ‘operator*’ (operand types are ‘FramedRotation<main()::B, main()::A>’ and ‘FramedVector<main()::C>’)
     auto not_allowed = R * vec_c;
                          ^
main.cpp:40:24: note: candidate: template<class To, class From> const FramedVector<To> operator*(FramedRotation<To, From>, const FramedVector<From>&)
 const FramedVector<To> operator*(const FramedRotation<To, From> R,
                        ^
main.cpp:40:24: note:   template argument deduction/substitution failed:
main.cpp:81:28: note:   deduced conflicting types for parameter ‘From’ (‘main()::A’ and ‘main()::C’)
     auto not_allowed = R * vec_c;
```
Clang writes
```cpp
main.cpp:81:26: error: invalid operands to binary expression ('FramedRotation<B, A>' and 'FramedVector<C>')
    auto not_allowed = R * vec_c;
                       ~ ^ ~~~~~
main.cpp:40:24: note: candidate template ignored: deduced conflicting types for parameter 'From' ('A' vs. 'C')
const FramedVector<To> operator*(const FramedRotation<To, From> R,
```
While these message give plenty of information, a programmer might be confused, especially by gcc's version.

One way to make error messages more friendly is to enforce correctness through static asserts, instead of limiting what is defined.
```cpp
template<typename To, typename From, typename VectorFrame>
const FramedVector<To> operator*(const FramedRotation<To, From> R,
                                 const FramedVector<VectorFrame> &v) {
    static_assert(std::is_same<From, VectorFrame>::value,
            "The vector must match the From frame of the rotation!");
    return FramedVector<To>{R.value() * v.value()};
};
```
Now the error message is
```cpp
main.cpp:50:5: error: static_assert failed "The vector must match the From frame of the rotation!"
    static_assert(std::is_same<From, VectorFrame>::value,
    ^             ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
main.cpp:81:26: note: in instantiation of function template specialization 'operator*<B, A, C>' requested here
    auto not_allowed = R * vec_c;
                         ^
```

### But what if I want to do something weird?

Yes, you can do something weird, but only if you're explicit about it. We add a small piece of code, an explicit conversion operator, to the `FramedVector` class. Then you could write

```cpp
    // R transforms from A to B, but vec_c is in frame C
    // Pretend vec_c is in frame A
    auto allowed = R * static_cast<FramedVector<A>>(vec_c);
```

### What are the reference frame types?
In the above example, they are just any type. So, technically you could define something silly like `Rotation<int, char>`. It is possible to give the reference frame types some additional meaning, but that would make things more complicated.


### What do we need for actual implementation?

In a real implementation we would want
- Vectors to have 3 designators, not 1
- More operations defined
- Integration with an actual Vector class

While the behind-the-scenes implementation will necessarily be more complex than the example, the usage should not be. Remember the goal for the usage is to write something like `Vector<ImuFrame, ImuFrame, CameraFrame> p` instead of `Vector3d I_p_I_C`.

The implementation questions are

- What operations are allowed?
    - Are there just Vector quantities, or different ones for, say, translation and anugular velocity?
- What syntax is desired?
    - Are three designators always needed?
    - How to deal with, e.g. time indexes? (Several options)
- How to enforce correctness and produce errors? Implicit (specialization) or explicit (static_assert)?
- How to integrate into geometry classes? Inherit or wrap? _Since we already have a wrapper Rotation class, can conveniently just add it into the wrapper_
- Simple template implementation as above, or fancy-pants Boost.MPL or Boost.Units-based library? _Probably simple implementation_
- Automatic conversion? A `tf`-like transform tree? _No_

### Possible frustrations

- If you know that the A Frame and the B frame are oriented the same way, you may normally just add vectors between them
 `A_p_CA = A_p_BA + B_p_CB`. But that's not true in general, and the system won't allow that. You would need to define a rotation. But this may not be a bad thing, as it requires explicity stating the assumption that there is no rotation between the frames. Some helpers may be desired, like IdentityRotation<B,A>.
 - Some people may want the decorators present on _all_ appearances of the variable (e.g. in a variable name like `A_p_CA`). My system by design puts the reference frame information in the type, not (necessarily) the name. (Note most IDEs will readily show the type for any appearance of a variable)
 
