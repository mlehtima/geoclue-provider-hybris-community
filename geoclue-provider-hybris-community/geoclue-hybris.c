/*
 * Geoclue-provider-hybris
 * geoclue-hybris.c - A hybris-based GPS-provider
 *
 * Author: Matti Lehtimäki <matti.lehtimaki@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include <config.h>

#include <string.h>

#include <android-config.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <getopt.h>
#include <math.h>

#include <dbus/dbus.h>
#include <dbus/dbus-glib-bindings.h>
#include <dbus/dbus-glib-lowlevel.h>

#include <syslog.h>

#include <hardware/gps.h>

#include <geoclue/gc-provider.h>
#include <geoclue/geoclue-error.h>
#include <geoclue/gc-iface-geoclue.h>
#include <geoclue/gc-iface-position.h>
#include <geoclue/gc-iface-satellite.h>
#include <geoclue/gc-iface-velocity.h>

#define GEOCLUE_TYPE_HYBRIS (geoclue_hybris_get_type ())
#define GEOCLUE_HYBRIS(obj) (G_TYPE_CHECK_INSTANCE_CAST ((obj), GEOCLUE_TYPE_HYBRIS, GeoclueHybris))

typedef struct {
    GcProvider parent;
    GMainLoop *loop;

    char *owner;
    int last_timestamp;
    double last_altitude;
    double last_bearing;
    double last_latitude;
    double last_longitude;
    double last_speed;
    GeoclueAccuracy *last_accuracy;
    GeocluePositionFields last_pos_fields;
    GeoclueVelocityFields last_velo_fields;
    GeoclueStatus last_status;
    GHashTable *connections;
} GeoclueHybris;

typedef struct {
    GcProviderClass parent_class;
} GeoclueHybrisClass;

static void geoclue_hybris_init (GeoclueHybris *obj);
static void geoclue_hybris_geoclue_init (GcIfaceGeoclueClass *iface);
static void geoclue_hybris_position_init (GcIfacePositionClass *iface);
static void geoclue_hybris_satellite_init (GcIfaceSatelliteClass *iface);
static void geoclue_hybris_velocity_init (GcIfaceVelocityClass *iface);
static void geoclue_hybris_update_position (GeoclueHybris *hybris, GpsLocation* location);
static void geoclue_hybris_update_velocity (GeoclueHybris *hybris, GpsLocation* location);
static void geoclue_hybris_update_satellites (GeoclueHybris *hybris, GpsSvStatus* sv_info);
static void geoclue_hybris_update_status (GeoclueHybris *hybris, GeoclueStatus status);

G_DEFINE_TYPE_WITH_CODE (GeoclueHybris, geoclue_hybris, GC_TYPE_PROVIDER,
                         G_IMPLEMENT_INTERFACE (GC_TYPE_IFACE_GEOCLUE,
                                                geoclue_hybris_geoclue_init)
                         G_IMPLEMENT_INTERFACE (GC_TYPE_IFACE_POSITION,
                                                geoclue_hybris_position_init)
                         G_IMPLEMENT_INTERFACE (GC_TYPE_IFACE_SATELLITE,
                                                geoclue_hybris_satellite_init)
                         G_IMPLEMENT_INTERFACE (GC_TYPE_IFACE_VELOCITY,
                                                geoclue_hybris_velocity_init))

GeoclueHybris *hybris = NULL;

/* Hybris GPS */

const GpsInterface* gps = NULL;

static const GpsInterface*
get_gps_interface()
{
    int error;
    hw_module_t* module;
    const GpsInterface* interface = NULL;
    struct gps_device_t *device;

    error = hw_get_module(GPS_HARDWARE_MODULE_ID, (hw_module_t const**)&module);

    if (!error)
    {
        error = module->methods->open(module, GPS_HARDWARE_MODULE_ID, (struct hw_device_t **) &device);

        if (!error)
        {
            interface = device->get_gps_interface(device);
        }
    }
    else
    {
        exit(1);
    }

    return interface;
}

static void
location_callback(GpsLocation* location)
{
    geoclue_hybris_update_status (hybris, GEOCLUE_STATUS_AVAILABLE);
    geoclue_hybris_update_position (hybris, location);
    geoclue_hybris_update_velocity (hybris, location);
}

static void
status_callback(GpsStatus* status)
{
    switch (status->status)
    {
        case GPS_STATUS_NONE:
        geoclue_hybris_update_status (hybris, GEOCLUE_STATUS_UNAVAILABLE);
        break;
        case GPS_STATUS_SESSION_BEGIN:
        break;
        case GPS_STATUS_SESSION_END:
        break;
        case GPS_STATUS_ENGINE_ON:
        geoclue_hybris_update_status (hybris, GEOCLUE_STATUS_ACQUIRING);
        break;
        case GPS_STATUS_ENGINE_OFF:
        geoclue_hybris_update_status (hybris, GEOCLUE_STATUS_UNAVAILABLE);
        break;
        default:
        break;
    }
}

static void
sv_status_callback(GpsSvStatus* sv_info)
{
    geoclue_hybris_update_satellites (hybris, sv_info);
}

static void
nmea_callback(GpsUtcTime timestamp, const char* nmea, int length)
{
    /* do nothing */
}

static void
set_capabilities_callback(uint32_t capabilities)
{
    /* do nothing */
}

static void
acquire_wakelock_callback()
{
    /* do nothing */
}

static void
release_wakelock_callback()
{
    /* do nothing */
}

struct ThreadWrapperContext {
    void (*func)(void *);
    void *user_data;
};

static void *
thread_wrapper_context_main_func(void *user_data)
{
  struct ThreadWrapperContext *ctx = (struct ThreadWrapperContext *)user_data;

  ctx->func(ctx->user_data);

  free(ctx);

  return NULL;
}

static pthread_t
create_thread_callback(const char* name, void (*start)(void *), void* arg)
{
  pthread_t thread_id;
  int error = 0;

  /* Wrap thread function, so we can return void * to pthread and log start/end of thread */
  struct ThreadWrapperContext *ctx = calloc(1, sizeof(struct ThreadWrapperContext));
  ctx->func = start;
  ctx->user_data = arg;

  /* Do not use a pthread_attr_t (we'd have to take care of bionic/glibc differences) */
  error = pthread_create(&thread_id, NULL, thread_wrapper_context_main_func, ctx);

  if(error != 0)
    return 0;

  return thread_id;
}

/* Hybris GPS callbacks */
GpsCallbacks callbacks = {
  sizeof(GpsCallbacks),
  location_callback,
  status_callback,
  sv_status_callback,
  nmea_callback,
  set_capabilities_callback,
  acquire_wakelock_callback,
  release_wakelock_callback,
  create_thread_callback,
};

/* Geoclue interfaces implementations */

static gboolean
equal_or_nan (double a, double b)
{
    if (isnan (a) && isnan (b)) {
        return TRUE;
    }
    return a == b;
}

/* General  */

static void
geoclue_hybris_update_status (GeoclueHybris *hybris, GeoclueStatus status)
{
    if (status != hybris->last_status) {
        hybris->last_status = status;
        /* make position and velocity invalid if no fix */
        if (status != GEOCLUE_STATUS_AVAILABLE) {
            hybris->last_pos_fields = GEOCLUE_POSITION_FIELDS_NONE;
            hybris->last_velo_fields = GEOCLUE_VELOCITY_FIELDS_NONE;
        }
        gc_iface_geoclue_emit_status_changed (GC_IFACE_GEOCLUE (hybris),
                                              status);
    }
}

static gboolean
geoclue_hybris_get_status (GcIfaceGeoclue *iface,
                          GeoclueStatus  *status,
                          GError        **error)
{
    GeoclueHybris *hybris = GEOCLUE_HYBRIS (iface);

    *status = hybris->last_status;

    return TRUE;
}

static gboolean
set_options (GcIfaceGeoclue *gc,
             GHashTable     *options,
             GError        **error)
{
    return TRUE;
}

/* Deinitialization */

static void
shutdown (GcProvider *provider)
{
    GeoclueHybris *hybris = GEOCLUE_HYBRIS (provider);

    g_main_loop_quit (hybris->loop);
}

static void
geoclue_hybris_dispose (GObject *obj)
{
    ((GObjectClass *) geoclue_hybris_parent_class)->dispose (obj);
}

static void
finalize (GObject *obj)
{
    GeoclueHybris *hybris = GEOCLUE_HYBRIS (obj);

    if (gps)
    {
        gps->stop();
        gps->cleanup();
        gps = NULL;
    }
    geoclue_accuracy_free (hybris->last_accuracy);
    g_hash_table_destroy (hybris->connections);

    ((GObjectClass *) geoclue_hybris_parent_class)->finalize (obj);
}

/* Position interface */

static void
geoclue_hybris_update_position (GeoclueHybris *hybris, GpsLocation* location)
{
    if (equal_or_nan (location->latitude, hybris->last_latitude) &&
        equal_or_nan (location->longitude, hybris->last_longitude) &&
        equal_or_nan (location->altitude, hybris->last_altitude)) {
        /* position has not changed */
        return;
    }

    hybris->last_latitude = location->latitude;
    hybris->last_longitude = location->longitude;
    hybris->last_altitude = location->altitude;
    hybris->last_timestamp = (int)(location->timestamp/1000+0.5);
    hybris->last_pos_fields = GEOCLUE_POSITION_FIELDS_LATITUDE | GEOCLUE_POSITION_FIELDS_LONGITUDE | GEOCLUE_POSITION_FIELDS_ALTITUDE;
    geoclue_accuracy_set_details (hybris->last_accuracy,
                                  GEOCLUE_ACCURACY_LEVEL_DETAILED,
                                  location->accuracy, location->accuracy);

    hybris->last_pos_fields = GEOCLUE_POSITION_FIELDS_NONE;
    hybris->last_pos_fields |= (isnan (location->latitude)) ?
                             0 : GEOCLUE_POSITION_FIELDS_LATITUDE;
    hybris->last_pos_fields |= (isnan (location->longitude)) ?
                             0 : GEOCLUE_POSITION_FIELDS_LONGITUDE;
    hybris->last_pos_fields |= (isnan (location->altitude)) ?
                             0 : GEOCLUE_POSITION_FIELDS_ALTITUDE;

    GeoclueAccuracy *accuracy = geoclue_accuracy_new (GEOCLUE_ACCURACY_LEVEL_DETAILED, location->accuracy, location->accuracy);
    gc_iface_position_emit_position_changed
        (GC_IFACE_POSITION (hybris),
         GEOCLUE_POSITION_FIELDS_LATITUDE | GEOCLUE_POSITION_FIELDS_LONGITUDE | GEOCLUE_POSITION_FIELDS_ALTITUDE,
         (int)(location->timestamp/1000+0.5),
         location->latitude, location->longitude, location->altitude,
         accuracy);

    geoclue_accuracy_free (accuracy);
}

static gboolean
get_position (GcIfacePosition *gc,
              GeocluePositionFields *fields,
              int                   *timestamp,
              double                *latitude,
              double                *longitude,
              double                *altitude,
              GeoclueAccuracy      **accuracy,
              GError               **error)
{
    *timestamp = (int)(hybris->last_timestamp+0.5);
    *fields =  hybris->last_pos_fields;
    *accuracy = geoclue_accuracy_copy (hybris->last_accuracy);
    *latitude = hybris->last_latitude;
    *longitude = hybris->last_longitude;
    *altitude = hybris->last_altitude;
    return TRUE;
}

/* Velocity interface */

static void
geoclue_hybris_update_velocity (GeoclueHybris *hybris, GpsLocation* location)
{
    if (equal_or_nan (location->speed, hybris->last_speed) &&
        equal_or_nan (location->bearing, hybris->last_bearing)) {
        /* velocity has not changed */
        return;
    }

    hybris->last_speed = location->speed;
    hybris->last_bearing = location->bearing;

    hybris->last_velo_fields = GEOCLUE_VELOCITY_FIELDS_NONE;
    hybris->last_velo_fields |= (isnan (hybris->last_bearing)) ?
        0 : GEOCLUE_VELOCITY_FIELDS_DIRECTION;
    hybris->last_velo_fields |= (isnan (hybris->last_speed)) ?
        0 : GEOCLUE_VELOCITY_FIELDS_SPEED;

    gc_iface_velocity_emit_velocity_changed
        (GC_IFACE_VELOCITY (hybris), hybris->last_velo_fields,
         (int)(hybris->last_timestamp+0.5),
         hybris->last_speed, hybris->last_bearing, 0);
}

static gboolean
get_velocity (GcIfaceVelocity       *gc,
              GeoclueVelocityFields *fields,
              int                   *timestamp,
              double                *speed,
              double                *direction,
              double                *climb,
              GError               **error)
{
    GeoclueHybris *hybris = GEOCLUE_HYBRIS (gc);

    *timestamp = (int)(hybris->last_timestamp+0.5);
    *speed = hybris->last_speed;
    *direction = hybris->last_bearing;
    *climb = 0;
    *fields = hybris->last_velo_fields;

    return TRUE;
}

/* Satellite interface */

static void
geoclue_hybris_update_satellites (GeoclueHybris *hybris, GpsSvStatus* sv_info)
{
    //TODO
}

static gboolean
get_satellite (GcIfaceSatellite *gc,
               int              *timestamp,
               int              *satellite_used,
               int              *satellite_visible,
               GArray          **used_prn,
               GPtrArray       **sat_info,
               GError          **error)
{
    //TODO
}

static gboolean
get_last_satellite (GcIfaceSatellite *gc,
                    int              *timestamp,
                    int              *satellite_used,
                    int              *satellite_visible,
                    GArray          **used_prn,
                    GPtrArray       **sat_info,
                    GError          **error)
{
    //TODO
}

/* Geoclue interface */

static gboolean
get_provider_info (GcIfaceGeoclue  *gc,
                   gchar          **name,
                   gchar          **description,
                   GError         **error)
{
    if (name) {
        *name = g_strdup ("Hybris");
    }
    if (description) {
        *description = g_strdup ("Hybris GPS provider");
    }
    if (error) {
        *error = NULL;
    }

    return TRUE;
}

static void
add_reference (GcIfaceGeoclue        *gc,
               DBusGMethodInvocation *context)
{
    char *sender;
    int *pcount;

    /* Update the hash of open connections */
    sender = dbus_g_method_get_sender (context);
    pcount = g_hash_table_lookup (hybris->connections, sender);
    if (!pcount) {
        pcount = g_malloc0 (sizeof (int));
        g_hash_table_insert (hybris->connections, sender, pcount);
    }
    (*pcount)++;
    if (g_hash_table_size (hybris->connections) == 1 && *pcount == 1) {
        hybris->owner = strdup(sender);
        gps->start();
    }
}

static void
remove_reference (GcIfaceGeoclue        *gc,
                  DBusGMethodInvocation *context)
{
    char *sender;
    int *pcount;

    sender = dbus_g_method_get_sender (context);
    pcount = g_hash_table_lookup (hybris->connections, sender);
    if (!pcount) {
        return;
    }

    (*pcount)--;
    if (*pcount == 0) {
        g_hash_table_remove (hybris->connections, sender);
    }
    if (g_hash_table_size (hybris->connections) == 0 || strcmp(sender, hybris->owner) == 0) {
        finalize((GObject *)hybris);
        g_main_loop_quit (hybris->loop);
    }
}

/* Initialization */

static void
geoclue_hybris_class_init (GeoclueHybrisClass *klass)
{
    GcProviderClass *p_class = (GcProviderClass *)klass;
    GObjectClass *o_class = (GObjectClass *)klass;

    o_class->finalize = finalize;
    p_class->shutdown = shutdown;
    p_class->set_options = set_options;
    p_class->get_status = geoclue_hybris_get_status;

    o_class->dispose = geoclue_hybris_dispose;
}

static void
geoclue_hybris_init (GeoclueHybris *hybris)
{
    gc_provider_set_details (GC_PROVIDER (hybris),
                            "org.freedesktop.Geoclue.Providers.Hybris",
                            "/org/freedesktop/Geoclue/Providers/Hybris",
                            "Hybris", "Hybris GPS provider");

    int initok = 0;
    hybris->last_accuracy = geoclue_accuracy_new (GEOCLUE_ACCURACY_LEVEL_NONE, 0, 0);
    hybris->last_latitude = 1.0;
    hybris->last_longitude = 1.0;
    hybris->last_altitude = 1.0;
    hybris->last_speed = 1.0;
    hybris->last_bearing = 1.0;
    hybris->last_timestamp = time(NULL);
    hybris->last_pos_fields = GEOCLUE_POSITION_FIELDS_NONE;
    hybris->last_velo_fields = GEOCLUE_VELOCITY_FIELDS_NONE;
    hybris->connections = g_hash_table_new_full (g_str_hash, g_str_equal,
                                                 g_free, g_free);

    gps = get_gps_interface();

    initok = gps->init(&callbacks);

    /* need to be done before starting gps or no info will come out */
    if (!initok)
        gps->set_position_mode(GPS_POSITION_MODE_MS_BASED,
                               GPS_POSITION_RECURRENCE_PERIODIC, 1000, 0, 0);
    else
        gps->set_position_mode(GPS_POSITION_MODE_STANDALONE,
                               GPS_POSITION_RECURRENCE_PERIODIC, 1000, 0, 0);

    geoclue_hybris_update_status (hybris, GEOCLUE_STATUS_ACQUIRING);
}

static void
geoclue_hybris_geoclue_init (GcIfaceGeoclueClass *iface)
{
    iface->get_provider_info = get_provider_info;
    iface->add_reference = add_reference;
    iface->remove_reference = remove_reference;
}

static void
geoclue_hybris_position_init (GcIfacePositionClass *iface)
{
    iface->get_position = get_position;
}

static void
geoclue_hybris_velocity_init (GcIfaceVelocityClass *iface)
{
    iface->get_velocity = get_velocity;
}

static void
geoclue_hybris_satellite_init (GcIfaceSatelliteClass *iface)
{
    //TODO
    //iface->get_satellite = get_satellite;
    //iface->get_last_satellite = get_last_satellite;
}

int
main()
{
    g_type_init();
    hybris = g_object_new (GEOCLUE_TYPE_HYBRIS, NULL);

    hybris->loop = g_main_loop_new (NULL, TRUE);

    g_main_loop_run (hybris->loop);

    g_main_loop_unref (hybris->loop);
    g_object_unref (hybris);

    return 0;
}
