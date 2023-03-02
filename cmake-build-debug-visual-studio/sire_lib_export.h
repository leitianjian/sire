
#ifndef SIRE_API_H
#define SIRE_API_H

#ifdef SIRE_LIB_STATIC_DEFINE
#  define SIRE_API
#  define SIRE_LIB_NO_EXPORT
#else
#  ifndef SIRE_API
#    ifdef sire_lib_EXPORTS
        /* We are building this library */
#      define SIRE_API __declspec(dllexport)
#    else
        /* We are using this library */
#      define SIRE_API __declspec(dllimport)
#    endif
#  endif

#  ifndef SIRE_LIB_NO_EXPORT
#    define SIRE_LIB_NO_EXPORT 
#  endif
#endif

#ifndef SIRE_LIB_DEPRECATED
#  define SIRE_LIB_DEPRECATED __declspec(deprecated)
#endif

#ifndef SIRE_LIB_DEPRECATED_EXPORT
#  define SIRE_LIB_DEPRECATED_EXPORT SIRE_API SIRE_LIB_DEPRECATED
#endif

#ifndef SIRE_LIB_DEPRECATED_NO_EXPORT
#  define SIRE_LIB_DEPRECATED_NO_EXPORT SIRE_LIB_NO_EXPORT SIRE_LIB_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef SIRE_LIB_NO_DEPRECATED
#    define SIRE_LIB_NO_DEPRECATED
#  endif
#endif

#endif /* SIRE_API_H */
