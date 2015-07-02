Name: geoclue-provider-hybris-community
Version: 0.12.99
Release: 1
Summary: Geoinformation Service Hybris Provider
Group: System/Libraries
URL: http://www.freedesktop.org/wiki/Software/GeoClue/
License: LGPLv2
Source: %{name}-%{version}.tar.gz
Conflicts: geoclue-provider-hybris
BuildRequires: pkgconfig(glib-2.0)
BuildRequires: pkgconfig(gobject-2.0)
BuildRequires: pkgconfig(dbus-glib-1)
BuildRequires: pkgconfig(libxml-2.0)
BuildRequires: pkgconfig(gio-2.0)
BuildRequires: pkgconfig(glib-2.0)
BuildRequires: pkgconfig(geoclue)
BuildRequires: pkgconfig(android-headers)
BuildRequires: pkgconfig(libhardware)
BuildRequires: autoconf
BuildRequires: libxslt

%description
%{summary}.

%files
%defattr(-,root,root,-)
%{_datadir}/dbus-1/services/org.freedesktop.Geoclue.Providers.Hybris.service
%{_datadir}/geoclue-providers/geoclue-hybris.provider
%{_libexecdir}/geoclue-hybris

%prep
%setup -q -n %{name}-%{version}/%{name}


%build
autoreconf -vfi
%configure --enable-static=no
make %{?_smp_mflags}


%install
make DESTDIR=%{buildroot} install
rm -f %{buildroot}/%{_libdir}/*.la

%post -p /sbin/ldconfig

%postun
/sbin/ldconfig
if [ $1 -eq 0 ]; then
    /usr/bin/glib-compile-schemas %{_datadir}/glib-2.0/schemas &>/dev/null || :
fi

%posttrans
/usr/bin/glib-compile-schemas %{_datadir}/glib-2.0/schemas &>/dev/null || :

