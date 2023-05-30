#ifndef SOEM_RSL_EXPORT_H
#define SOEM_RSL_EXPORT_H

#ifdef SOEM_RSL_STATIC_DEFINE
#  define SOEM_RSL_EXPORT
#  define SOEM_RSL_NO_EXPORT
#else
#  ifndef SOEM_RSL_EXPORT
#    ifdef soem_rsl_EXPORTS
        /* We are building this library */
#      define SOEM_RSL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define SOEM_RSL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef SOEM_RSL_NO_EXPORT
#    define SOEM_RSL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef SOEM_RSL_DEPRECATED
#  define SOEM_RSL_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef SOEM_RSL_DEPRECATED_EXPORT
#  define SOEM_RSL_DEPRECATED_EXPORT SOEM_RSL_EXPORT SOEM_RSL_DEPRECATED
#endif

#ifndef SOEM_RSL_DEPRECATED_NO_EXPORT
#  define SOEM_RSL_DEPRECATED_NO_EXPORT SOEM_RSL_NO_EXPORT SOEM_RSL_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef SOEM_RSL_NO_DEPRECATED
#    define SOEM_RSL_NO_DEPRECATED
#  endif
#endif

#endif /* SOEM_RSL_EXPORT_H */
