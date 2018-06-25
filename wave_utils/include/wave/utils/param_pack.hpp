/// https://stackoverflow.com/questions/20162903/template-parameter-packs-access-nth-type-and-nth-element#29753388
/// somehow there doesn't seem to be an easier compile-time way

#ifndef WAVE_PARAM_PACK_HPP
#define WAVE_PARAM_PACK_HPP

#include <type_traits>
#include <tuple>

namespace wave {

template<std::size_t index, typename T, typename... Ts>
inline constexpr typename std::enable_if<index==0, T>::type
get(T&& t, Ts&&... ) {
    return t;
}

template<std::size_t index, typename T, typename... Ts>
inline constexpr typename std::enable_if<(index > 0) && index <= sizeof...(Ts),
        typename std::tuple_element<index, std::tuple<T, Ts...>>::type>::type
get(T&& , Ts&&... ts) {
    return get<index-1>(std::forward<Ts>(ts)...);
}

}

#endif //WAVE_PARAM_PACK_HPP
