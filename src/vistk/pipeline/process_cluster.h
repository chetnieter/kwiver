/*ckwg +5
 * Copyright 2012 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#ifndef VISTK_PIPELINE_PROCESS_CLUSTER_H
#define VISTK_PIPELINE_PROCESS_CLUSTER_H

#include "pipeline-config.h"

#include "process.h"

#include <boost/scoped_ptr.hpp>

/**
 * \file process_cluster.h
 *
 * \brief Header for \link vistk::process_cluster process clusters\endlink.
 */

namespace vistk
{

/**
 * \class process_cluster process_cluster.h <vistk/pipeline/process_cluster.h>
 *
 * \brief A pre-built collection of processes.
 *
 * \ingroup base_classes
 */
class VISTK_PIPELINE_EXPORT process_cluster
  : public process
{
  public:
    /**
     * \brief The processes in the cluster.
     *
     * \returns The processes in the cluster.
     */
    processes_t processes() const;
    /**
     * \brief Input mappings for the cluster.
     *
     * \returns The input mappings for the cluster.
     */
    connections_t input_mappings() const;
    /**
     * \brief Output mappings for the cluster.
     *
     * \returns The output mappings for the cluster.
     */
    connections_t output_mappings() const;
    /**
     * \brief Internal connections between processes in the cluster.
     *
     * \returns The internal connections between processes in the cluster.
     */
    connections_t internal_connections() const;

    /// A property which indicates that the process is a cluster of processes.
    static property_t const property_cluster;
  protected:
    /**
     * \brief Constructor.
     *
     * \warning Configuration errors must \em not throw exceptions here.
     *
     * \param config Contains configuration for the process.
     */
    process_cluster(config_t const& config);
    /**
     * \brief Destructor.
     */
    virtual ~process_cluster();

    /**
     * \brief Map a configuration value to a process.
     *
     * \param key The key on the cluster.
     * \param name_ The process to map the configuration to.
     * \param mapped_key The key to map the configuration to.
     */
    void map_config(config::key_t const& key, name_t const& name_, config::key_t const& mapped_key);
    /**
     * \brief Add a process to the cluster.
     *
     * \throws duplicate_process_name_exception Thrown when a process named \p name already exists.
     *
     * \param name_ The name of the process.
     * \param type_ The type of the process.
     * \param conf The base configuration to use.
     */
    void add_process(name_t const& name_, type_t const& type_, config_t const& config = config::empty_config());
    /**
     * \brief Map a port to an input on the cluster.
     *
     * \throws no_such_process_exception Thrown when \p name does not exist in the cluster.
     *
     * \param port The port on the cluster.
     * \param name_ The name of the process to map the input to.
     * \param mapped_port The port on the process to map the input to.
     */
    void input_map(port_t const& port, name_t const& name_, port_t const& mapped_port);
    /**
     * \brief Map a port to an output on the cluster.
     *
     * \throws no_such_process_exception Thrown when \p name does not exist in the cluster.
     *
     * \param port The port on the cluster.
     * \param name_ The name of the process to map the output to.
     * \param mapped_port The port on the process to map the output to.
     */
    void output_map(port_t const& port, name_t const& name_, port_t const& mapped_port);
    /**
     * \brief Connect processes within the cluster.
     *
     * \throws no_such_process_exception Thrown when either \p upstream_name or \p downstream_name do not exist in the cluster.
     *
     * \param upstream_name The upstream process name.
     * \param upstream_port The upstream process port.
     * \param downstream_name The downstream process name.
     * \param downstream_port The downstream process port.
     */
    void connect(name_t const& upstream_name, port_t const& upstream_port,
                 name_t const& downstream_name, port_t const& downstream_port);

    /**
     * \brief Pre-connection initialization for subclasses.
     */
    void _configure();

    /**
     * \brief Post-connection initialization for subclasses.
     */
    void _init();

    /**
     * \brief Reset logic for subclasses.
     */
    void _reset();

    /**
     * \brief A stub implementation to ensure that clusters should not be stepped.
     *
     * \throws process_exception Always thrown since clusters should not be stepped.
     */
    void _step();

    /**
     * \brief Subclass property query method.
     *
     * \returns Properties on the subclass.
     */
    virtual properties_t _properties() const;
  private:
    class priv;
    boost::scoped_ptr<priv> d;
};

}

#endif // VISTK_PIPELINE_PROCESS_CLUSTER_H
