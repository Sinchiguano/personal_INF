#ifndef UTILITIES_HPP
#define UTILITIES_HPP


#include <iostream>
#include <sstream>
#include <functional>
#include <algorithm>
#include <random>
#include <unistd.h>

template <class V> auto idxs(V const &v) {
  std::vector<int> result;
  for (int i = 0; i < v.size(); ++i)
    result.push_back(i);
  return result;
}

template<class T, class Compare>
constexpr const T& clamp( const T& v, const T& lo, const T& hi, Compare comp ) {
    return assert( !comp(hi, lo) ), comp(v, lo) ? lo : comp(hi, v) ? hi : v;
}

template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi ) {
    return clamp( v, lo, hi, std::less<T>() );
}

inline double lin_interp( double ratio, double lo, double hi ) {
    return lo + ratio*(hi-lo);
}

inline std::default_random_engine &get_random_engine() {
    std::random_device rd;
    static std::default_random_engine eng(rd());
    return eng;
}

inline double rand_uniform (double min, double max) {
    std::uniform_real_distribution<double> randomGenerator(min, max);
    return randomGenerator(get_random_engine());
}

template <typename T> T parse(std::string const &str) {
    T val;
    std::istringstream sstr(str);
    sstr >> val;
    return val;
}


template <class F>
struct OnDtor {
    F f_;
    OnDtor(F f) : f_(f) { }
    ~OnDtor() {f_();}
};

template <class F>
OnDtor<F> makeOnDtor(F f) {
    return OnDtor<F>(f);
}

template <class T>
std::string Ttos(T t) {
    std::stringstream ss;
    ss << t;
    return ss.str();
}

inline double sq(double x) {
    return x*x;
}

template <class Collection> void print_v (Collection const &v) {
    for (auto const &el : v) {
        std::cout << el << ", " << std::endl;
    }
}

template <class C, class... Args>
void push_back(C &c, Args const &...args) {
    using val_type = typename std::remove_reference<decltype(*c.begin())>::type;
    val_type val {args...};
    c.push_back(val);
}

inline void delay( unsigned long ms ) {
    usleep( ms * 1000 );
}

struct RandGenerator {
  std::random_device rd_;
  std::mt19937 gen_;
  RandGenerator() : rd_{}, gen_{rd_()} {}
};

template <class DistributionT>
class DistributionGenerator
{
public:
  DistributionGenerator(DistributionT const &d)
      : rand_gen{}, distribution{d}
  {
  }
  auto operator() () {
    return distribution(rand_gen.gen_);
  }

private:
  RandGenerator rand_gen;
  DistributionT distribution;
};

template <class DT>
auto makeDistributionGenerator(DT const &d) {
    return DistributionGenerator<DT> (d);
}

namespace func {

template <typename CollectionTo, typename CollectionFrom, typename unop>
CollectionTo map(CollectionFrom const &col, unop op) {
    CollectionTo to;
    std::transform(col.begin(), col.end(), std::back_inserter(to), op);
    return to;
}

template <typename From, typename unop>
auto v_map(std::vector<From> const &col, unop op) ->
std::vector<decltype(op(col.front()))>
{
    std::vector<decltype(op(col.front()))> to;
    std::transform(col.begin(), col.end(), std::back_inserter(to), op);
    return to;
}

template <class Collection>
auto max_element(Collection const &in) -> decltype(in.begin()){
    return std::max_element(in.begin(), in.end());
}

template <class Collection>
auto min_element(Collection const &in) -> decltype(in.begin()){
    return std::min_element(in.begin(), in.end());
}

template <class Collection, class F>
auto min_element(Collection const &in, F &&op) -> decltype(in.begin()){
    return std::min_element(in.begin(), in.end(), op);
}

template <class Collection, class F>
Collection filter(Collection const &in, F &&op) {
    Collection to;
    std::copy_if(in.begin(), in.end(), std::back_inserter(to), op);
    return to;
}

template <class CollectionFrom, class CollectionCmp, class F>
CollectionFrom filter(CollectionFrom const &from,
        CollectionCmp const &cmp, F &&op) {
    CollectionFrom to;
    for (int i = 0; i < from.size(); ++i) {
        if (op(cmp[i]))
        { to.push_back(from[i]); }
    }
    return to;
}

template<typename T>
std::vector<T> flatten(const std::vector<std::vector<T>> &orig)
{
    std::vector<T> ret;
    for(const auto &v: orig)
        ret.insert(ret.end(), v.begin(), v.end());
    return ret;
}

}

#endif // UTILITIES_HPP
