#ifndef CNOID_IRSL_XXX_PLUGIN_EXPORTDECL_H
#define CNOID_IRSL_XXX_PLUGIN_EXPORTDECL_H

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_XXXPLUGIN_DLLIMPORT __declspec(dllimport)
#  define CNOID_XXXPLUGIN_DLLEXPORT __declspec(dllexport)
#  define CNOID_XXXPLUGIN_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_XXXPLUGIN_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_XXXPLUGIN_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_XXXPLUGIN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_XXXPLUGIN_DLLIMPORT
#   define CNOID_XXXPLUGIN_DLLEXPORT
#   define CNOID_XXXPLUGIN_DLLLOCAL
#  endif
# endif

# ifdef CNOID_XXXPLUGIN_STATIC
#  define CNOID_XXXPLUGIN_DLLAPI
#  define CNOID_XXXPLUGIN_LOCAL
# else
#  ifdef CnoidIRSLXXXPlugin_EXPORTS
#   define CNOID_XXXPLUGIN_DLLAPI CNOID_XXXPLUGIN_DLLEXPORT
#  else
#   define CNOID_XXXPLUGIN_DLLAPI CNOID_XXXPLUGIN_DLLIMPORT
#  endif
#  define CNOID_XXXPLUGIN_LOCAL CNOID_XXXPLUGIN_DLLLOCAL
# endif

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_XXXPLUGIN_DLLAPI

#endif // CNOID_IRSL_XXX_PLUGIN_EXPORTDECL_H
