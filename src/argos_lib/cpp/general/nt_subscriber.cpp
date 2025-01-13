/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/nt_subscriber.h"

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "fmt/format.h"

using argos_lib::NTSubscriber;

NTSubscriber::NTSubscriber(const std::string& tableName) : m_tableName{tableName} {}

NTSubscriber::~NTSubscriber() {
  // Need to release all handles
  for (auto listenerIt = m_ntListeners.begin(); listenerIt != m_ntListeners.end(); ++listenerIt) {
    nt::RemoveListener(*listenerIt);
  }
  for (auto entryIt = m_ntEntries.begin(); entryIt != m_ntEntries.end(); ++entryIt) {
    nt::ReleaseEntry(*entryIt);
  }
}

void NTSubscriber::AddMonitor(const std::string& keyName,
                              std::function<void(double)> onUpdateCallback,
                              const double defaultValue,
                              const bool forceUpdate) {
  NT_Topic topic = nt::GetTopic(NT_GetDefaultInstance(), fmt::format("{}/{}", m_tableName, keyName));
  m_ntEntries.push_back(nt::GetEntry(topic, NT_DOUBLE, "double"));
  if (forceUpdate) {
    nt::SetDouble(m_ntEntries.back(), defaultValue);
  } else {
    nt::SetDefaultDouble(m_ntEntries.back(), defaultValue);
  }

  nt::AddListener(m_ntEntries.back(), nt::EventFlags::kValueAll, [onUpdateCallback](const nt::Event& e) {
    onUpdateCallback(e.GetValueEventData()->value.GetDouble());
  });
}
