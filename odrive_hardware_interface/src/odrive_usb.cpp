// Copyright 2021 Factor Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "odrive_hardware_interface/odrive_usb.hpp"

#include <cstdlib>

namespace odrive
{
ODriveUSB::ODriveUSB()
{
  libusb_context_ = NULL;
  sequence_number_ = 0;
}

ODriveUSB::~ODriveUSB()
{
  for (auto it = odrive_map_.begin(); it != odrive_map_.end(); it++) {
    libusb_release_interface(it->second, 2);
    libusb_close(it->second);
  }
  odrive_map_.clear();

  if (libusb_context_) {
    libusb_exit(libusb_context_);
    libusb_context_ = NULL;
  }
}

int ODriveUSB::init(const std::vector<std::vector<int64_t>> & serial_numbers)
{
  int ret = libusb_init(&libusb_context_);
  if (ret != LIBUSB_SUCCESS) {
    return ret;
  }

  libusb_device ** device_list;
  ssize_t device_count = libusb_get_device_list(libusb_context_, &device_list);
  if (device_count < 0) {
    return device_count;
  }

  for (ssize_t i = 0; i < device_count; ++i) {
    libusb_device * device = device_list[i];
    libusb_device_descriptor descriptor;

    if (libusb_get_device_descriptor(device, &descriptor) != LIBUSB_SUCCESS) {
      continue;
    }

    if (
      descriptor.idVendor == ODRIVE_USB_VENDORID && descriptor.idProduct == ODRIVE_USB_PRODUCTID) {
      const int bus_number = libusb_get_bus_number(device);
      const int device_address = libusb_get_device_address(device);

      libusb_device_handle * device_handle;
      ret = libusb_open(device, &device_handle);
      if (ret != LIBUSB_SUCCESS) {
        std::cerr << "Skipping ODrive at bus " << bus_number << " address " << device_address
                  << ": open failed with " << libusb_error_name(ret) << std::endl;
        continue;
      }
      if (
        (libusb_kernel_driver_active(device_handle, 2) != LIBUSB_SUCCESS) &&
        (libusb_detach_kernel_driver(device_handle, 2) != LIBUSB_SUCCESS)) {
        std::cerr << "Skipping ODrive at bus " << bus_number << " address " << device_address
                  << ": could not detach kernel driver" << std::endl;
        libusb_close(device_handle);
        continue;
      }
      ret = libusb_claim_interface(device_handle, 2);
      if (ret != LIBUSB_SUCCESS) {
        // LIBUSB_ERROR_BUSY is expected for a board another hardware component already owns.
        if (ret != LIBUSB_ERROR_BUSY) {
          std::cerr << "Skipping ODrive at bus " << bus_number << " address " << device_address
                    << ": claim failed with " << libusb_error_name(ret) << std::endl;
        }
        libusb_close(device_handle);
        continue;
      }
      uint64_t serial_number;
      ret = read(device_handle, SERIAL_NUMBER, serial_number);
      if (ret != LIBUSB_SUCCESS) {
        std::cerr << "Skipping ODrive at bus " << bus_number << " address " << device_address
                  << ": serial number read failed with " << libusb_error_name(ret) << std::endl;
        libusb_release_interface(device_handle, 2);
        libusb_close(device_handle);
        continue;
      }
      odrive_map_.insert(std::pair<int64_t, libusb_device_handle *>(-serial_number, device_handle));
    }
  }

  libusb_free_device_list(device_list, 1);
  if (!odrive_map_.size()) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  bool all_requested_odrives_found = true;
  for (const std::vector<int64_t> & component_serial_numbers : serial_numbers) {
    for (int64_t requested_serial_number : component_serial_numbers) {
      // A zero serial number means "whichever ODrive is attached", only unambiguous with one board.
      if (!requested_serial_number && odrive_map_.size() == 1) {
        requested_serial_number = std::abs(odrive_map_.begin()->first);
      }
      if (odrive_map_.count(requested_serial_number)) {
        continue;
      }
      auto it = odrive_map_.find(-requested_serial_number);
      if (it == odrive_map_.end()) {
        std::cerr << "ODrive " << std::hex << requested_serial_number << std::dec << " not found"
                  << std::endl;
        all_requested_odrives_found = false;
        continue;
      }
      odrive_map_.insert(std::pair<int64_t, libusb_device_handle *>(-it->first, it->second));
      std::cout << "Connected to ODrive " << std::hex << -it->first << std::dec << std::endl;
      odrive_map_.erase(it);
    }
  }

  // On failure every board is released, otherwise this context would keep a board that another
  // hardware component needs to claim.
  for (auto it = odrive_map_.begin(); it != odrive_map_.end();) {
    if (it->first < 0 || !all_requested_odrives_found) {
      libusb_release_interface(it->second, 2);
      libusb_close(it->second);
      it = odrive_map_.erase(it);
    } else {
      it++;
    }
  }

  return all_requested_odrives_found ? LIBUSB_SUCCESS : LIBUSB_ERROR_NO_DEVICE;
}

template <typename T>
int ODriveUSB::read(int64_t & serial_number, short endpoint_id, T & value)
{
  auto it = serial_number ? odrive_map_.find(serial_number) : odrive_map_.begin();
  if (it == odrive_map_.end()) {
    return LIBUSB_ERROR_NO_DEVICE;
  }
  return read(it->second, endpoint_id, value);
}

template <typename T>
int ODriveUSB::read(libusb_device_handle * odrive_handle, short endpoint_id, T & value)
{
  bytes request_payload;
  bytes response_payload;

  int ret = endpointOperation(
    odrive_handle, endpoint_id, sizeof(value), request_payload, response_payload, 1);
  if (ret != LIBUSB_SUCCESS) {
    return ret;
  }
  if (response_payload.size() < sizeof(value)) {
    return LIBUSB_ERROR_IO;
  }

  std::memcpy(&value, response_payload.data(), sizeof(value));

  return LIBUSB_SUCCESS;
}

template <typename T>
int ODriveUSB::write(int64_t & serial_number, short endpoint_id, const T & value)
{
  auto it = serial_number ? odrive_map_.find(serial_number) : odrive_map_.begin();
  if (it == odrive_map_.end()) {
    return LIBUSB_ERROR_NO_DEVICE;
  }
  return write(it->second, endpoint_id, value);
}

template <typename T>
int ODriveUSB::write(libusb_device_handle * odrive_handle, short endpoint_id, const T & value)
{
  bytes request_payload;
  bytes response_payload;

  for (size_t i = 0; i < sizeof(value); i++) {
    request_payload.emplace_back(((unsigned char *)&value)[i]);
  }

  return endpointOperation(odrive_handle, endpoint_id, 0, request_payload, response_payload, 1);
}

int ODriveUSB::call(int64_t & serial_number, short endpoint_id)
{
  auto it = serial_number ? odrive_map_.find(serial_number) : odrive_map_.begin();
  if (it == odrive_map_.end()) {
    return LIBUSB_ERROR_NO_DEVICE;
  }
  return call(it->second, endpoint_id);
}

int ODriveUSB::call(libusb_device_handle * odrive_handle, short endpoint_id)
{
  bytes request_payload;
  bytes response_payload;

  return endpointOperation(odrive_handle, endpoint_id, 0, request_payload, response_payload, 1);
}

int ODriveUSB::endpointOperation(
  libusb_device_handle * odrive_handle, short endpoint_id, short response_size,
  bytes request_payload, bytes & response_payload, bool MSB)
{
  int ret = LIBUSB_ERROR_OTHER;
  for (int attempt = 0; attempt < ODRIVE_TRANSFER_ATTEMPTS; attempt++) {
    ret = transferPacket(
      odrive_handle, endpoint_id, response_size, request_payload, response_payload, MSB);
    const bool transient_error =
      ret == LIBUSB_ERROR_PIPE || ret == LIBUSB_ERROR_IO || ret == LIBUSB_ERROR_OVERFLOW;
    if (!transient_error) {
      break;
    }
  }

  return ret;
}

int ODriveUSB::transferPacket(
  libusb_device_handle * odrive_handle, short endpoint_id, short response_size,
  const bytes & request_payload, bytes & response_payload, bool MSB)
{
  if (MSB) {
    endpoint_id |= 0x8000;
  }
  sequence_number_ = (sequence_number_ + 1) & 0x7fff;
  const uint16_t sequence_number = sequence_number_;

  bytes request_packet = encodePacket(sequence_number, endpoint_id, response_size, request_payload);

  int transferred = 0;
  int ret = libusb_bulk_transfer(
    odrive_handle, ODRIVE_OUT_ENDPOINT, request_packet.data(), request_packet.size(), &transferred,
    0);
  if (ret == LIBUSB_ERROR_PIPE) {
    libusb_clear_halt(odrive_handle, ODRIVE_OUT_ENDPOINT);
  }
  if (ret != LIBUSB_SUCCESS) {
    return ret;
  }
  if (transferred != static_cast<int>(request_packet.size())) {
    return LIBUSB_ERROR_IO;
  }

  if (!MSB) {
    return LIBUSB_SUCCESS;
  }

  // A response to an earlier request, from a failed attempt or a process that died mid-transfer,
  // can still be queued on the ODrive; the sequence number tells it apart from this one.
  for (int stale_response = 0; stale_response <= ODRIVE_MAX_STALE_RESPONSES; stale_response++) {
    unsigned char response_data[ODRIVE_MAX_PACKET_SIZE];
    ret = libusb_bulk_transfer(
      odrive_handle, ODRIVE_IN_ENDPOINT, response_data, ODRIVE_MAX_PACKET_SIZE, &transferred, 0);
    if (ret == LIBUSB_ERROR_PIPE) {
      libusb_clear_halt(odrive_handle, ODRIVE_IN_ENDPOINT);
    }
    if (ret != LIBUSB_SUCCESS) {
      return ret;
    }
    if (transferred < 2) {
      continue;
    }

    const uint16_t response_sequence_number = response_data[0] | (response_data[1] << 8);
    if (response_sequence_number != (sequence_number | 0x8000)) {
      continue;
    }

    bytes response_packet(response_data, response_data + transferred);
    response_payload = decodePacket(response_packet);
    return LIBUSB_SUCCESS;
  }

  return LIBUSB_ERROR_IO;
}

bytes ODriveUSB::encodePacket(
  short sequence_number, short endpoint_id, short response_size, const bytes & request_payload)
{
  bytes packet;

  packet.emplace_back((sequence_number >> 0) & 0xFF);
  packet.emplace_back((sequence_number >> 8) & 0xFF);
  packet.emplace_back((endpoint_id >> 0) & 0xFF);
  packet.emplace_back((endpoint_id >> 8) & 0xFF);
  packet.emplace_back((response_size >> 0) & 0xFF);
  packet.emplace_back((response_size >> 8) & 0xFF);

  for (uint8_t b : request_payload) {
    packet.emplace_back(b);
  }

  short crc = ((endpoint_id & 0x7fff) == 0) ? ODRIVE_PROTOCOL_VERSION : json_crc;
  packet.emplace_back((crc >> 0) & 0xFF);
  packet.emplace_back((crc >> 8) & 0xFF);

  return packet;
}

bytes ODriveUSB::decodePacket(bytes & response_packet)
{
  bytes payload;

  for (bytes::size_type i = 2; i < response_packet.size(); ++i) {
    payload.emplace_back(response_packet[i]);
  }

  return payload;
}

template int ODriveUSB::read(int64_t &, short, bool &);
template int ODriveUSB::read(int64_t &, short, float &);
template int ODriveUSB::read(int64_t &, short, int32_t &);
template int ODriveUSB::read(int64_t &, short, uint8_t &);
template int ODriveUSB::read(int64_t &, short, uint16_t &);
template int ODriveUSB::read(int64_t &, short, uint32_t &);
template int ODriveUSB::read(int64_t &, short, uint64_t &);

template int ODriveUSB::write(int64_t &, short, const bool &);
template int ODriveUSB::write(int64_t &, short, const float &);
template int ODriveUSB::write(int64_t &, short, const int32_t &);
template int ODriveUSB::write(int64_t &, short, const uint8_t &);
template int ODriveUSB::write(int64_t &, short, const uint16_t &);
template int ODriveUSB::write(int64_t &, short, const uint32_t &);
template int ODriveUSB::write(int64_t &, short, const uint64_t &);
}  // namespace odrive
