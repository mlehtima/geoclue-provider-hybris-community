libexec_PROGRAMS = \
	geoclue-hybris

geoclue_hybris_SOURCES = \
	geoclue-hybris.c

geoclue_hybris_CFLAGS = \
	-I$(top_srcdir) \
	-I$(top_builddir) \
	$(GEOCLUE_CFLAGS) \
	-pthread \
	$(DROIDHEADERS_CFLAGS) \
	$(HYBRIS_CFLAGS)

geoclue_hybris_LDADD = \
	$(GEOCLUE_LIBS) \
	$(HYBRIS_LIBS)

geoclue_hybris_LDFLAGS = \
	-pthread

providersdir = $(datadir)/geoclue-providers
providers_DATA = geoclue-hybris.provider

servicedir = $(DBUS_SERVICES_DIR)
service_in_files = org.freedesktop.Geoclue.Providers.Hybris.service.in
service_DATA = $(service_in_files:.service.in=.service)

$(service_DATA): $(service_in_files) Makefile
	$(AM_V_GEN) sed -e "s|\@libexecdir\@|$(libexecdir)|" $< > $@

EXTRA_DIST = 			\
	$(service_in_files)	\
	$(providers_DATA)

DISTCLEANFILES = \
	$(service_DATA)
