#include "storagehandler.hpp"

#include "fruread.hpp"
#include "read_fru_data.hpp"
#include "selutility.hpp"
#include "sensorhandler.hpp"
#include "storageaddsel.hpp"

#include <arpa/inet.h>
#include <mapper.h>
#include <systemd/sd-bus.h>

#include <algorithm>
#include <boost/beast/core/span.hpp>
#include <boost/process.hpp>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <ipmid/api.hpp>
#include <ipmid/utils.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/server.hpp>
#include <sdrutils.hpp>
#include <string>
#include <string_view>
#include <variant>
#include <xyz/openbmc_project/Common/error.hpp>

void register_netfn_storage_functions() __attribute__((constructor));

unsigned int g_sel_time = 0xFFFFFFFF;
namespace ipmi
{
namespace sensor
{
extern const IdInfoMap sensors;
} // namespace sensor
} // namespace ipmi

extern const FruMap frus;
constexpr uint8_t eventDataSize = 3;
namespace
{
constexpr auto TIME_INTERFACE = "xyz.openbmc_project.Time.EpochTime";
constexpr auto BMC_TIME_PATH = "/xyz/openbmc_project/time/bmc";
constexpr auto DBUS_PROPERTIES = "org.freedesktop.DBus.Properties";
constexpr auto PROPERTY_ELAPSED = "Elapsed";

} // namespace


#ifndef JOURNAL_SEL
namespace cache
{
/*
 * This cache contains the object paths of the logging entries sorted in the
 * order of the filename(numeric order). The cache is initialized by
 * invoking readLoggingObjectPaths with the cache as the parameter. The
 * cache is invoked in the execution of the Get SEL info and Delete SEL
 * entry command. The Get SEL Info command is typically invoked before the
 * Get SEL entry command, so the cache is utilized for responding to Get SEL
 * entry command. The cache is invalidated by clearing after Delete SEL
 * entry and Clear SEL command.
 */
ipmi::sel::ObjectPaths paths;

} // namespace cache
#endif

using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;
using namespace phosphor::logging;
using namespace ipmi::fru;

/**
 * @enum Device access mode
 */
enum class AccessMode
{
    bytes, ///< Device is accessed by bytes
    words  ///< Device is accessed by words
};

#ifdef JOURNAL_SEL
namespace ipmi::sel::erase_time
{
static constexpr const char* selEraseTimestamp = "/var/lib/ipmi/sel_erase_time";

void save()
{
    // open the file, creating it if necessary
    int fd = open(selEraseTimestamp, O_WRONLY | O_CREAT | O_CLOEXEC, 0644);
    if (fd < 0)
    {
        std::cerr << "Failed to open file\n";
        return;
    }

    // update the file timestamp to the current time
    if (futimens(fd, NULL) < 0)
    {
        std::cerr << "Failed to update timestamp: "
                  << std::string(strerror(errno));
    }
    close(fd);
}

int get()
{
    struct stat st;
    // default to an invalid timestamp
    int timestamp = ::ipmi::sel::invalidTimeStamp;

    int fd = open(selEraseTimestamp, O_RDWR | O_CLOEXEC, 0644);
    if (fd < 0)
    {
        return timestamp;
    }

    if (fstat(fd, &st) >= 0)
    {
        timestamp = st.st_mtime;
    }

    return timestamp;
}
} // namespace ipmi::sel::erase_time

static int fromHexStr(const std::string hexStr, std::vector<uint8_t>& data)
{
    for (unsigned int i = 0; i < hexStr.size(); i += 2)
    {
        try
        {
            data.push_back(static_cast<uint8_t>(
                std::stoul(hexStr.substr(i, 2), nullptr, 16)));
        }
        catch (std::invalid_argument& e)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
            return -1;
        }
        catch (std::out_of_range& e)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
            return -1;
        }
    }
    return 0;
}

static const std::filesystem::path selLogDir = "/var/log";
static const std::string selLogFilename = "ipmi_sel";

static bool getSELLogFiles(std::vector<std::filesystem::path>& selLogFiles)
{
    // Loop through the directory looking for ipmi_sel log files
    for (const std::filesystem::directory_entry& dirEnt :
         std::filesystem::directory_iterator(selLogDir))
    {
        std::string filename = dirEnt.path().filename();
        if (boost::starts_with(filename, selLogFilename))
        {
            // If we find an ipmi_sel log file, save the path
            selLogFiles.emplace_back(selLogDir /
                                     filename);
        }
    }
    // As the log files rotate, they are appended with a ".#" that is higher for
    // the older logs. Since we don't expect more than 10 log files, we
    // can just sort the list to get them in order from newest to oldest
    std::sort(selLogFiles.begin(), selLogFiles.end());

    return !selLogFiles.empty();
}

static int countSELEntries()
{
    // Get the list of ipmi_sel log files
    std::vector<std::filesystem::path> selLogFiles;
    if (!getSELLogFiles(selLogFiles))
    {
        return 0;
    }
    int numSELEntries = 0;
    // Loop through each log file and count the number of logs
    for (const std::filesystem::path& file : selLogFiles)
    {
        std::ifstream logStream(file);
        if (!logStream.is_open())
        {
            continue;
        }

        std::string line;
        while (std::getline(logStream, line))
        {
            numSELEntries++;
        }
    }
    return numSELEntries;
}

static bool findSELEntry(const int recordID,
                         const std::vector<std::filesystem::path>& selLogFiles,
                         std::string& entry)
{
    // Record ID is the first entry field following the timestamp. It is
    // preceded by a space and followed by a comma
    std::string search = " " + std::to_string(recordID) + ",";

    // Loop through the ipmi_sel log entries
    for (const std::filesystem::path& file : selLogFiles)
    {
        std::ifstream logStream(file);
        if (!logStream.is_open())
        {
            continue;
        }

        while (std::getline(logStream, entry))
        {
            // Check if the record ID matches
            if (entry.find(search) != std::string::npos)
            {
                return true;
            }
        }
    }
    return false;
}

static uint16_t
    getNextRecordID(const uint16_t recordID,
                    const std::vector<std::filesystem::path>& selLogFiles)
{
    uint16_t nextRecordID = recordID + 1;
    std::string entry;
    if (findSELEntry(nextRecordID, selLogFiles, entry))
    {
        return nextRecordID;
    }
    else
    {
        return ipmi::sel::lastEntry;
    }
}

static int getFileTimestamp(const std::filesystem::path& file)
{
    struct stat st;

    if (stat(file.c_str(), &st) >= 0)
    {
        return st.st_mtime;
    }
    return ::ipmi::sel::invalidTimeStamp;
}

using systemEventType = std::tuple<
    uint32_t, // Timestamp
    uint16_t, // Generator ID
    uint8_t,  // EvM Rev
    uint8_t,  // Sensor Type
    uint8_t,  // Sensor Number
    uint7_t,  // Event Type
    bool,     // Event Direction
    std::array<uint8_t, ipmi::sel::systemEventSize>>; // Event Data
using oemTsEventType = std::tuple<
    uint32_t,                                                   // Timestamp
    std::array<uint8_t, ipmi::sel::oemTsEventSize>>; // Event Data
using oemEventType =
    std::array<uint8_t, ipmi::sel::oemEventSize>; // Event Data

ipmi::RspType<uint16_t, // Next Record ID
              uint16_t, // Record ID
              uint8_t,  // Record Type
              std::variant<systemEventType, oemTsEventType,
                           oemEventType>> // Record Content
    ipmiStorageGetSELEntry(uint16_t reservationID, uint16_t targetID,
                           uint8_t offset, uint8_t size)
{
    // Only support getting the entire SEL record. If a partial size or non-zero
    // offset is requested, return an error
    if (offset != 0 || size != ipmi::sel::entireRecord)
    {
        return ipmi::responseRetBytesUnavailable();
    }

    // Check the reservation ID if one is provided or required (only if the
    // offset is non-zero)
    if (reservationID != 0 || offset != 0)
    {
        if (!checkSELReservation(reservationID))
        {
            return ipmi::responseInvalidReservationId();
        }
    }

    // Get the ipmi_sel log files
    std::vector<std::filesystem::path> selLogFiles;
    if (!getSELLogFiles(selLogFiles))
    {
        return ipmi::responseSensorInvalid();
    }

    std::string targetEntry;

    if (targetID == ipmi::sel::firstEntry)
    {
        // The first entry will be at the top of the oldest log file
        std::ifstream logStream(selLogFiles.back());
        if (!logStream.is_open())
        {
            return ipmi::responseUnspecifiedError();
        }

        if (!std::getline(logStream, targetEntry))
        {
            return ipmi::responseUnspecifiedError();
        }
    }
    else if (targetID == ipmi::sel::lastEntry)
    {
        // The last entry will be at the bottom of the newest log file
        std::ifstream logStream(selLogFiles.front());
        if (!logStream.is_open())
        {
            return ipmi::responseUnspecifiedError();
        }

        std::string line;
        while (std::getline(logStream, line))
        {
            targetEntry = line;
        }
    }
    else
    {
        if (!findSELEntry(targetID, selLogFiles, targetEntry))
        {
            return ipmi::responseSensorInvalid();
        }
    }

    // The format of the ipmi_sel message is "<Timestamp>
    // <ID>,<Type>,<EventData>,[<Generator ID>,<Path>,<Direction>]".
    // First get the Timestamp
    size_t space = targetEntry.find_first_of(" ");
    if (space == std::string::npos)
    {
        return ipmi::responseUnspecifiedError();
    }
    std::string entryTimestamp = targetEntry.substr(0, space);
    // Then get the log contents
    size_t entryStart = targetEntry.find_first_not_of(" ", space);
    if (entryStart == std::string::npos)
    {
        return ipmi::responseUnspecifiedError();
    }
    std::string_view entry(targetEntry);
    entry.remove_prefix(entryStart);
    // Use split to separate the entry into its fields
    std::vector<std::string> targetEntryFields;
    boost::split(targetEntryFields, entry, boost::is_any_of(","),
                 boost::token_compress_on);
    if (targetEntryFields.size() < 3)
    {
        return ipmi::responseUnspecifiedError();
    }
    std::string& recordIDStr = targetEntryFields[0];
    std::string& recordTypeStr = targetEntryFields[1];
    std::string& eventDataStr = targetEntryFields[2];

    uint16_t recordID;
    uint8_t recordType;
    try
    {
        recordID = std::stoul(recordIDStr);
        recordType = std::stoul(recordTypeStr, nullptr, 16);
    }
    catch (const std::invalid_argument&)
    {
        return ipmi::responseUnspecifiedError();
    }
    uint16_t nextRecordID = getNextRecordID(recordID, selLogFiles);
    std::vector<uint8_t> eventDataBytes;
    if (fromHexStr(eventDataStr, eventDataBytes) < 0)
    {
        return ipmi::responseUnspecifiedError();
    }

    if (recordType == ipmi::sel::systemEvent)
    {
        // Get the timestamp
        std::tm timeStruct = {};
        std::istringstream entryStream(entryTimestamp);

        uint32_t timestamp = ipmi::sel::invalidTimeStamp;
        if (entryStream >> std::get_time(&timeStruct, "%Y-%m-%dT%H:%M:%S"))
        {
            timestamp = std::mktime(&timeStruct);
        }

        // Set the event message revision
        uint8_t evmRev = ipmi::sel::eventMsgRev;

        uint16_t generatorID = 0;
        uint8_t sensorType = 0;
        uint16_t sensorAndLun = 0;
        uint8_t sensorNum = 0xFF;
        uint7_t eventType = 0;
        bool eventDir = 0;
        // System type events should have six fields
        if (targetEntryFields.size() >= 6)
        {
            std::string& generatorIDStr = targetEntryFields[3];
            std::string& sensorPath = targetEntryFields[4];
            std::string& eventDirStr = targetEntryFields[5];

            // Get the generator ID
            try
            {
                generatorID = std::stoul(generatorIDStr, nullptr, 16);
            }
            catch (const std::invalid_argument&)
            {
                std::cerr << "Invalid Generator ID\n";
            }

            // Get the sensor type, sensor number, and event type for the sensor
            sensorType = getSensorTypeFromPath(sensorPath);
            sensorAndLun = getSensorNumberFromPath(sensorPath);
            sensorNum = static_cast<uint8_t>(sensorAndLun);
            generatorID |= sensorAndLun >> 8;
            eventType = getSensorEventTypeFromPath(sensorPath);

            // Get the event direction
            try
            {
                eventDir = std::stoul(eventDirStr) ? 0 : 1;
            }
            catch (const std::invalid_argument&)
            {
                std::cerr << "Invalid Event Direction\n";
            }
        }

        // Only keep the eventData bytes that fit in the record
        std::array<uint8_t, ipmi::sel::systemEventSize> eventData{};
        std::copy_n(eventDataBytes.begin(),
                    std::min(eventDataBytes.size(), eventData.size()),
                    eventData.begin());

        return ipmi::responseSuccess(
            nextRecordID, recordID, recordType,
            systemEventType{timestamp, generatorID, evmRev, sensorType,
                            sensorNum, eventType, eventDir, eventData});
    }
    else if (recordType >= ipmi::sel::oemTsEventFirst &&
             recordType <= ipmi::sel::oemTsEventLast)
    {
        // Get the timestamp
        std::tm timeStruct = {};
        std::istringstream entryStream(entryTimestamp);

        uint32_t timestamp = ipmi::sel::invalidTimeStamp;
        if (entryStream >> std::get_time(&timeStruct, "%Y-%m-%dT%H:%M:%S"))
        {
            timestamp = std::mktime(&timeStruct);
        }

        // Only keep the bytes that fit in the record
        std::array<uint8_t, ipmi::sel::oemTsEventSize> eventData{};
        std::copy_n(eventDataBytes.begin(),
                    std::min(eventDataBytes.size(), eventData.size()),
                    eventData.begin());

        return ipmi::responseSuccess(nextRecordID, recordID, recordType,
                                     oemTsEventType{timestamp, eventData});
    }
    else if (recordType >= ipmi::sel::oemEventFirst)
    {
        // Only keep the bytes that fit in the record
        std::array<uint8_t, ipmi::sel::oemEventSize> eventData{};
        std::copy_n(eventDataBytes.begin(),
                    std::min(eventDataBytes.size(), eventData.size()),
                    eventData.begin());

        return ipmi::responseSuccess(nextRecordID, recordID, recordType,
                                     eventData);
    }

    return ipmi::responseUnspecifiedError();
}


ipmi::RspType<uint8_t> ipmiStorageClearSEL(ipmi::Context::ptr ctx,
                                           uint16_t reservationID,
                                           const std::array<uint8_t, 3>& clr,
                                           uint8_t eraseOperation)
{
    if (!checkSELReservation(reservationID))
    {
        return ipmi::responseInvalidReservationId();
    }

    static constexpr std::array<uint8_t, 3> clrExpected = {'C', 'L', 'R'};
    if (clr != clrExpected)
    {
        return ipmi::responseInvalidFieldRequest();
    }

    // Erasure status cannot be fetched, so always return erasure status as
    // `erase completed`.
    if (eraseOperation == ipmi::sel::getEraseStatus)
    {
        return ipmi::responseSuccess(ipmi::sel::eraseComplete);
    }

    // Check that initiate erase is correct
    if (eraseOperation != ipmi::sel::initiateErase)
    {
        return ipmi::responseInvalidFieldRequest();
    }

    // Per the IPMI spec, need to cancel any reservation when the SEL is
    // cleared
    cancelSELReservation();

    // Save the erase time
    ipmi::sel::erase_time::save();

    // Clear the SEL by deleting the log files
    std::vector<std::filesystem::path> selLogFiles;
    if (getSELLogFiles(selLogFiles))
    {
        for (const std::filesystem::path& file : selLogFiles)
        {
            std::error_code ec;
            std::filesystem::remove(file, ec);
        }
    }

    // Reload rsyslog so it knows to start new log files
    std::shared_ptr<sdbusplus::asio::connection> dbus = getSdBus();
    sdbusplus::message::message rsyslogReload = dbus->new_method_call(
        "org.freedesktop.systemd1", "/org/freedesktop/systemd1",
        "org.freedesktop.systemd1.Manager", "ReloadUnit");
    rsyslogReload.append("rsyslog.service", "replace");
    try
    {
        sdbusplus::message::message reloadResponse = dbus->call(rsyslogReload);
    }
    catch (sdbusplus::exception_t& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
    }

    return ipmi::responseSuccess(ipmi::sel::eraseComplete);
}



ipmi::RspType<uint8_t,  // SEL version
              uint16_t, // SEL entry count
              uint16_t, // free space
              uint32_t, // last add timestamp
              uint32_t, // last erase timestamp
              uint8_t>  // operation support
    ipmiStorageGetSelInfo()
{
    constexpr uint8_t selVersion = ipmi::sel::selVersion;
    uint16_t entries = countSELEntries();
    uint32_t addTimeStamp = getFileTimestamp(
        selLogDir / selLogFilename);
    uint32_t eraseTimeStamp = ipmi::sel::erase_time::get();
    constexpr uint8_t operationSupport =
        ipmi::sel::selOperationSupport;
    constexpr uint16_t freeSpace =
        0xffff; // Spec indicates that more than 64kB is free

    return ipmi::responseSuccess(selVersion, entries, freeSpace, addTimeStamp,
                                 eraseTimeStamp, operationSupport);
}
#else // JOURNAL_SEL not used
/** @brief implements the get SEL Info command
 *  @returns IPMI completion code plus response data
 *   - selVersion - SEL revision
 *   - entries    - Number of log entries in SEL.
 *   - freeSpace  - Free Space in bytes.
 *   - addTimeStamp - Most recent addition timestamp
 *   - eraseTimeStamp - Most recent erase timestamp
 *   - operationSupport - Reserve & Delete SEL operations supported
 */

ipmi::RspType<uint8_t,  // SEL revision.
              uint16_t, // number of log entries in SEL.
              uint16_t, // free Space in bytes.
              uint32_t, // most recent addition timestamp
              uint32_t, // most recent erase timestamp.

              bool,    // SEL allocation info supported
              bool,    // reserve SEL supported
              bool,    // partial Add SEL Entry supported
              bool,    // delete SEL supported
              uint3_t, // reserved
              bool     // overflow flag
              >
    ipmiStorageGetSelInfo()
{
    uint16_t entries = 0;
    // Most recent addition timestamp.
    uint32_t addTimeStamp = ipmi::sel::invalidTimeStamp;

    try
    {
        ipmi::sel::readLoggingObjectPaths(cache::paths);
    }
    catch (const sdbusplus::exception::SdBusError& e)
    {
        // No action if reading log objects have failed for this command.
        // readLoggingObjectPaths will throw exception if there are no log
        // entries. The command will be responded with number of SEL entries
        // as 0.
    }

    if (!cache::paths.empty())
    {
        entries = static_cast<uint16_t>(cache::paths.size());

        try
        {
            addTimeStamp = static_cast<uint32_t>(
                (ipmi::sel::getEntryTimeStamp(cache::paths.back()).count()));
        }
        catch (InternalFailure& e)
        {
        }
        catch (const std::runtime_error& e)
        {
            log<level::ERR>(e.what());
        }
    }

    constexpr uint8_t selVersion = ipmi::sel::selVersion;
    constexpr uint16_t freeSpace = 0xFFFF;
    constexpr uint32_t eraseTimeStamp = ipmi::sel::invalidTimeStamp;
    constexpr uint3_t reserved{0};

    return ipmi::responseSuccess(
        selVersion, entries, freeSpace, addTimeStamp, eraseTimeStamp,
        ipmi::sel::operationSupport::getSelAllocationInfo,
        ipmi::sel::operationSupport::reserveSel,
        ipmi::sel::operationSupport::partialAddSelEntry,
        ipmi::sel::operationSupport::deleteSel, reserved,
        ipmi::sel::operationSupport::overflow);
}

ipmi_ret_t getSELEntry(ipmi_netfn_t netfn, ipmi_cmd_t cmd,
                       ipmi_request_t request, ipmi_response_t response,
                       ipmi_data_len_t data_len, ipmi_context_t context)
{
    if (*data_len != sizeof(ipmi::sel::GetSELEntryRequest))
    {
        *data_len = 0;
        return IPMI_CC_REQ_DATA_LEN_INVALID;
    }

    auto requestData =
        reinterpret_cast<const ipmi::sel::GetSELEntryRequest*>(request);

    if (requestData->reservationID != 0)
    {
        if (!checkSELReservation(requestData->reservationID))
        {
            *data_len = 0;
            return IPMI_CC_INVALID_RESERVATION_ID;
        }
    }

    if (cache::paths.empty())
    {
        *data_len = 0;
        return IPMI_CC_SENSOR_INVALID;
    }

    ipmi::sel::ObjectPaths::const_iterator iter;

    // Check for the requested SEL Entry.
    if (requestData->selRecordID == ipmi::sel::firstEntry)
    {
        iter = cache::paths.begin();
    }
    else if (requestData->selRecordID == ipmi::sel::lastEntry)
    {
        iter = cache::paths.end();
    }
    else
    {
        std::string objPath = std::string(ipmi::sel::logBasePath) + "/" +
                              std::to_string(requestData->selRecordID);

        iter = std::find(cache::paths.begin(), cache::paths.end(), objPath);
        if (iter == cache::paths.end())
        {
            *data_len = 0;
            return IPMI_CC_SENSOR_INVALID;
        }
    }

    ipmi::sel::GetSELEntryResponse record{};

    // Convert the log entry into SEL record.
    try
    {
        record = ipmi::sel::convertLogEntrytoSEL(*iter);
    }
    catch (InternalFailure& e)
    {
        *data_len = 0;
        return IPMI_CC_UNSPECIFIED_ERROR;
    }
    catch (const std::runtime_error& e)
    {
        log<level::ERR>(e.what());
        *data_len = 0;
        return IPMI_CC_UNSPECIFIED_ERROR;
    }

    // Identify the next SEL record ID
    if (iter != cache::paths.end())
    {
        ++iter;
        if (iter == cache::paths.end())
        {
            record.nextRecordID = ipmi::sel::lastEntry;
        }
        else
        {
            namespace fs = std::filesystem;
            fs::path path(*iter);
            record.nextRecordID = static_cast<uint16_t>(
                std::stoul(std::string(path.filename().c_str())));
        }
    }
    else
    {
        record.nextRecordID = ipmi::sel::lastEntry;
    }

    if (requestData->readLength == ipmi::sel::entireRecord)
    {
        std::memcpy(response, &record, sizeof(record));
        *data_len = sizeof(record);
    }
    else
    {
        if (requestData->offset >= ipmi::sel::selRecordSize ||
            requestData->readLength > ipmi::sel::selRecordSize)
        {
            *data_len = 0;
            return IPMI_CC_INVALID_FIELD_REQUEST;
        }

        auto diff = ipmi::sel::selRecordSize - requestData->offset;
        auto readLength =
            std::min(diff, static_cast<int>(requestData->readLength));

        std::memcpy(response, &record.nextRecordID,
                    sizeof(record.nextRecordID));
        std::memcpy(static_cast<uint8_t*>(response) +
                        sizeof(record.nextRecordID),
                    &record.recordID + requestData->offset, readLength);
        *data_len = sizeof(record.nextRecordID) + readLength;
    }

    return IPMI_CC_OK;
}

/** @brief implements the delete SEL entry command
 * @request
 *   - reservationID; // reservation ID.
 *   - selRecordID;   // SEL record ID.
 *
 *  @returns ipmi completion code plus response data
 *   - Record ID of the deleted record
 */
ipmi::RspType<uint16_t // deleted record ID
              >
    deleteSELEntry(uint16_t reservationID, uint16_t selRecordID)
{

    namespace fs = std::filesystem;

    if (!checkSELReservation(reservationID))
    {
        return ipmi::responseInvalidReservationId();
    }

    // Per the IPMI spec, need to cancel the reservation when a SEL entry is
    // deleted
    cancelSELReservation();

    try
    {
        ipmi::sel::readLoggingObjectPaths(cache::paths);
    }
    catch (const sdbusplus::exception::SdBusError& e)
    {
        // readLoggingObjectPaths will throw exception if there are no error
        // log entries.
        return ipmi::responseSensorInvalid();
    }

    if (cache::paths.empty())
    {
        return ipmi::responseSensorInvalid();
    }

    ipmi::sel::ObjectPaths::const_iterator iter;
    uint16_t delRecordID = 0;

    if (selRecordID == ipmi::sel::firstEntry)
    {
        iter = cache::paths.begin();
        fs::path path(*iter);
        delRecordID = static_cast<uint16_t>(
            std::stoul(std::string(path.filename().c_str())));
    }
    else if (selRecordID == ipmi::sel::lastEntry)
    {
        iter = cache::paths.end();
        fs::path path(*iter);
        delRecordID = static_cast<uint16_t>(
            std::stoul(std::string(path.filename().c_str())));
    }
    else
    {
        std::string objPath = std::string(ipmi::sel::logBasePath) + "/" +
                              std::to_string(selRecordID);

        iter = std::find(cache::paths.begin(), cache::paths.end(), objPath);
        if (iter == cache::paths.end())
        {
            return ipmi::responseSensorInvalid();
        }
        delRecordID = selRecordID;
    }

    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
    std::string service;

    try
    {
        service = ipmi::getService(bus, ipmi::sel::logDeleteIntf, *iter);
    }
    catch (const std::runtime_error& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }

    auto methodCall = bus.new_method_call(service.c_str(), (*iter).c_str(),
                                          ipmi::sel::logDeleteIntf, "Delete");
    auto reply = bus.call(methodCall);
    if (reply.is_method_error())
    {
        return ipmi::responseUnspecifiedError();
    }

    // Invalidate the cache of dbus entry objects.
    cache::paths.clear();

    return ipmi::responseSuccess(delRecordID);
}

/** @brief implements the Clear SEL command
 * @request
 *   - reservationID   // Reservation ID.
 *   - clr             // char array { 'C'(0x43h), 'L'(0x4Ch), 'R'(0x52h) }
 *   - eraseOperation; // requested operation.
 *
 *  @returns ipmi completion code plus response data
 *   - erase status
 */

ipmi::RspType<uint8_t // erase status
              >
    clearSEL(uint16_t reservationID, const std::array<char, 3>& clr,
             uint8_t eraseOperation)
{
    static constexpr std::array<char, 3> clrOk = {'C', 'L', 'R'};
    if (clr != clrOk)
    {
        return ipmi::responseInvalidFieldRequest();
    }

    if (!checkSELReservation(reservationID))
    {
        return ipmi::responseInvalidReservationId();
    }

    /*
     * Erasure status cannot be fetched from DBUS, so always return erasure
     * status as `erase completed`.
     */
    if (eraseOperation == ipmi::sel::getEraseStatus)
    {
        return ipmi::responseSuccess(
            static_cast<uint8_t>(ipmi::sel::eraseComplete));
    }

    // Per the IPMI spec, need to cancel any reservation when the SEL is cleared
    cancelSELReservation();

    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
    ipmi::sel::ObjectPaths objectPaths;
    auto depth = 0;

    auto mapperCall =
        bus.new_method_call(ipmi::sel::mapperBusName, ipmi::sel::mapperObjPath,
                            ipmi::sel::mapperIntf, "GetSubTreePaths");
    mapperCall.append(ipmi::sel::logBasePath);
    mapperCall.append(depth);
    mapperCall.append(ipmi::sel::ObjectPaths({ipmi::sel::logEntryIntf}));

    try
    {
        auto reply = bus.call(mapperCall);
        if (reply.is_method_error())
        {
            return ipmi::responseSuccess(
                static_cast<uint8_t>(ipmi::sel::eraseComplete));
        }

        reply.read(objectPaths);
        if (objectPaths.empty())
        {
            return ipmi::responseSuccess(
                static_cast<uint8_t>(ipmi::sel::eraseComplete));
        }
    }
    catch (const sdbusplus::exception::SdBusError& e)
    {
        return ipmi::responseSuccess(
            static_cast<uint8_t>(ipmi::sel::eraseComplete));
    }

    std::string service;

    try
    {
        service = ipmi::getService(bus, ipmi::sel::logDeleteIntf,
                                   objectPaths.front());
    }
    catch (const std::runtime_error& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }

    for (const auto& iter : objectPaths)
    {
        auto methodCall = bus.new_method_call(
            service.c_str(), iter.c_str(), ipmi::sel::logDeleteIntf, "Delete");

        auto reply = bus.call(methodCall);
        if (reply.is_method_error())
        {
            return ipmi::responseUnspecifiedError();
        }
    }

    // Invalidate the cache of dbus entry objects.
    cache::paths.clear();
    return ipmi::responseSuccess(
        static_cast<uint8_t>(ipmi::sel::eraseComplete));
}
#endif

/** @brief implements the get SEL time command
 *  @returns IPMI completion code plus response data
 *   -current time
 */
ipmi::RspType<uint32_t> // current time
    ipmiStorageGetSelTime()
{
    using namespace std::chrono;
    uint64_t bmc_time_usec = 0;
    std::stringstream bmcTime;

    try
    {
        sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
        auto service = ipmi::getService(bus, TIME_INTERFACE, BMC_TIME_PATH);
        std::variant<uint64_t> value;

        // Get bmc time
        auto method = bus.new_method_call(service.c_str(), BMC_TIME_PATH,
                                          DBUS_PROPERTIES, "Get");

        method.append(TIME_INTERFACE, PROPERTY_ELAPSED);
        auto reply = bus.call(method);
        if (reply.is_method_error())
        {
            log<level::ERR>("Error getting time",
                            entry("SERVICE=%s", service.c_str()),
                            entry("PATH=%s", BMC_TIME_PATH));
            return ipmi::responseUnspecifiedError();
        }
        reply.read(value);
        bmc_time_usec = std::get<uint64_t>(value);
    }
    catch (InternalFailure& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }
    catch (const std::exception& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }

    bmcTime << "BMC time:"
            << duration_cast<seconds>(microseconds(bmc_time_usec)).count();
    log<level::DEBUG>(bmcTime.str().c_str());

    // Time is really long int but IPMI wants just uint32. This works okay until
    // the number of seconds since 1970 overflows uint32 size.. Still a whole
    // lot of time here to even think about that.
    return ipmi::responseSuccess(
        duration_cast<seconds>(microseconds(bmc_time_usec)).count());
}

/** @brief implements the set SEL time command
 *  @param selDeviceTime - epoch time
 *        -local time as the number of seconds from 00:00:00, January 1, 1970
 *  @returns IPMI completion code
 */
ipmi::RspType<> ipmiStorageSetSelTime(uint32_t selDeviceTime)
{
    using namespace std::chrono;
    microseconds usec{seconds(selDeviceTime)};

    try
    {
        sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
        auto service = ipmi::getService(bus, TIME_INTERFACE, BMC_TIME_PATH);
        std::variant<uint64_t> value{(uint64_t)usec.count()};

        // Set bmc time
        auto method = bus.new_method_call(service.c_str(), BMC_TIME_PATH,
                                          DBUS_PROPERTIES, "Set");

        method.append(TIME_INTERFACE, PROPERTY_ELAPSED, value);
        auto reply = bus.call(method);
        if (reply.is_method_error())
        {
            log<level::ERR>("Error setting time",
                            entry("SERVICE=%s", service.c_str()),
                            entry("PATH=%s", BMC_TIME_PATH));
            return ipmi::responseUnspecifiedError();
        }
    }
    catch (InternalFailure& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }
    catch (const std::exception& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }

    return ipmi::responseSuccess();
}

/** @brief implements the reserve SEL command
 *  @returns IPMI completion code plus response data
 *   - SEL reservation ID.
 */
ipmi::RspType<uint16_t> ipmiStorageReserveSel()
{
    return ipmi::responseSuccess(reserveSel());
}

#ifdef JOURNAL_SEL
#if 0
static void toHexStr(const boost::beast::span<uint8_t> bytes,
                     std::string& hexStr)
{
    std::stringstream stream;
    stream << std::hex << std::uppercase << std::setfill('0');
    for (const uint8_t& byte : bytes)
    {
        stream << std::setw(2) << static_cast<int>(byte);
    }
    hexStr = stream.str();
}

static inline bool defaultMessageHook(uint16_t recordID, uint8_t recordType,
                       uint32_t timestamp, uint16_t generatorID, uint8_t evmRev,
                       uint8_t sensorType, uint8_t sensorNum, uint8_t eventType,
                       std::array<uint8_t, eventDataSize> eventData)
{
    // Log the record as a default Redfish message instead of a SEL record

    // Save the raw IPMI string of the request
    std::string ipmiRaw;
    std::array selBytes = {static_cast<uint8_t>(recordID),
                           static_cast<uint8_t>(recordID >> 8),
                           recordType,
                           static_cast<uint8_t>(timestamp),
                           static_cast<uint8_t>(timestamp >> 8),
                           static_cast<uint8_t>(timestamp >> 16),
                           static_cast<uint8_t>(timestamp >> 24),
                           static_cast<uint8_t>(generatorID),
                           static_cast<uint8_t>(generatorID >> 8),
                           evmRev,
                           sensorType,
                           sensorNum,
                           eventType,
                           eventData[0],
                           eventData[1],
                           eventData[2]};

    toHexStr(boost::beast::span<uint8_t>(selBytes), ipmiRaw);

    static const std::string openBMCMessageRegistryVersion("0.1");
    std::string messageID =
        "OpenBMC." + openBMCMessageRegistryVersion + ".SELEntryAdded";

    std::vector<std::string> messageArgs;
    messageArgs.push_back(ipmiRaw);

    // Log the Redfish message to the journal with the appropriate metadata
    std::string journalMsg = "SEL Entry Added: " + ipmiRaw;
    std::string messageArgsString = boost::algorithm::join(messageArgs, ",");
    phosphor::logging::log<phosphor::logging::level::INFO>(
        journalMsg.c_str(),
        phosphor::logging::entry("REDFISH_MESSAGE_ID=%s", messageID.c_str()),
        phosphor::logging::entry("REDFISH_MESSAGE_ARGS=%s",
                                 messageArgsString.c_str()));

    return true;
}
#endif

ipmi::RspType<uint16_t> ipmiStorageAddSEL(
    uint16_t recordID, uint8_t recordType, uint32_t timestamp,
    uint16_t generatorID, uint8_t evmRev, uint8_t sensorType, uint8_t sensorNum,
    uint8_t eventType, std::array<uint8_t, eventDataSize> eventData)
{
    // Per the IPMI spec, need to cancel any reservation when a SEL entry is
    // added
    cancelSELReservation();
#if 0
    // Send this request to the Redfish hooks to log it as a Redfish message
    // instead.  There is no need to add it to the SEL, so just return success.
    defaultMessageHook(
        recordID, recordType, timestamp, generatorID, evmRev, sensorType,
        sensorNum, eventType, eventData);
#endif
    static constexpr char const* ipmiSELObject =
        "xyz.openbmc_project.Logging.IPMI";
    static constexpr char const* ipmiSELPath =
        "/xyz/openbmc_project/Logging/IPMI";
    static constexpr char const* ipmiSELAddInterface =
        "xyz.openbmc_project.Logging.IPMI";
    static const std::string ipmiSELAddMessage =
        "IPMI SEL entry logged using IPMI Add SEL Entry command.";

    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};

    if (recordType == ipmi::sel::systemEvent)
    {
        std::string sensorPath = getPathFromSensorNumber(sensorNum);
        if(sensorPath.length() == 0){
            return ipmi::responseSensorInvalid();
        }

        bool assert =
            (eventType & ipmi::sel::deassertionEvent) ? false : true;
        uint16_t genId = generatorID;
        sdbusplus::message::message writeSEL = bus.new_method_call(
            ipmiSELObject, ipmiSELPath, ipmiSELAddInterface, "IpmiSelAdd");
        writeSEL.append(ipmiSELAddMessage, sensorPath, eventData, assert,
                        genId);
        try
        {
            sdbusplus::message::message writeSELResp = bus.call(writeSEL);
            writeSELResp.read(recordID);
        }
        catch (sdbusplus::exception_t& e)
        {
            log<level::ERR>(e.what());
            return ipmi::responseUnspecifiedError();
        }
    }
    else if (recordType >= ipmi::sel::oemTsEventFirst &&
             recordType <= ipmi::sel::oemEventLast)
    {
        sdbusplus::message::message writeSEL = bus.new_method_call(
            ipmiSELObject, ipmiSELPath, ipmiSELAddInterface, "IpmiSelAddOem");
        writeSEL.append(ipmiSELAddMessage, eventData, recordType);
        try
        {
            sdbusplus::message::message writeSELResp = bus.call(writeSEL);
            writeSELResp.read(recordID);
        }
        catch (sdbusplus::exception_t& e)
        {
            log<level::ERR>(e.what());
            return ipmi::responseUnspecifiedError();
        }
    }
    else
    {
        return ipmi::responseUnspecifiedError();
    }

   // uint16_t responseID = 0xFFFF;
    return ipmi::responseSuccess(recordID);
}
#else  // JOURNAL_SEL not used
/** @brief implements the Add SEL entry command
 * @request
 *
 *   - recordID      ID used for SEL Record access
 *   - recordType    Record Type
 *   - timeStamp     Time when event was logged. LS byte first
 *   - generatorID   software ID if event was generated from
 *                   system software
 *   - evmRev        event message format version
 *   - sensorType    sensor type code for service that generated
 *                   the event
 *   - sensorNumber  number of sensors that generated the event
 *   - eventDir     event dir
 *   - eventData    event data field contents
 *
 *  @returns ipmi completion code plus response data
 *   - RecordID of the Added SEL entry
 */
ipmi::RspType<uint16_t // recordID of the Added SEL entry
              >
    ipmiStorageAddSEL(uint16_t recordID, uint8_t recordType, uint32_t timeStamp,
                      uint16_t generatorID, uint8_t evmRev, uint8_t sensorType,
                      uint8_t sensorNumber, uint8_t eventDir,
                      std::array<uint8_t, eventDataSize> eventData)
{
    // Per the IPMI spec, need to cancel the reservation when a SEL entry is
    // added
    cancelSELReservation();
    // Hostboot sends SEL with OEM record type 0xDE to indicate that there is
    // a maintenance procedure associated with eSEL record.
    static constexpr auto procedureType = 0xDE;
    if (recordType == procedureType)
    {
        // In the OEM record type 0xDE, byte 11 in the SEL record indicate the
        // procedure number.
        createProcedureLogEntry(sensorType);
    }

    return ipmi::responseSuccess(recordID);
}
#endif // JOURNAL_SEL

/** @brief implements the get FRU Inventory Area Info command
 *
 *  @returns IPMI completion code plus response data
 *   - FRU Inventory area size in bytes,
 *   - access bit
 **/
ipmi::RspType<uint16_t, // FRU Inventory area size in bytes,
              uint8_t   // access size (bytes / words)
              >
    ipmiStorageGetFruInvAreaInfo(uint8_t fruID)
{

    auto iter = frus.find(fruID);
    if (iter == frus.end())
    {
        return ipmi::responseSensorInvalid();
    }

    try
    {
        return ipmi::responseSuccess(
            static_cast<uint16_t>(getFruAreaData(fruID).size()),
            static_cast<uint8_t>(AccessMode::bytes));
    }
    catch (const InternalFailure& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }
}

/**@brief implements the Read FRU Data command
 * @param fruDeviceId - FRU device ID. FFh = reserved
 * @param offset      - FRU inventory offset to read
 * @param readCount   - count to read
 *
 * @return IPMI completion code plus response data
 * - returnCount - response data count.
 * - data        -  response data
 */
ipmi::RspType<uint8_t,              // count returned
              std::vector<uint8_t>> // FRU data
    ipmiStorageReadFruData(uint8_t fruDeviceId, uint16_t offset,
                           uint8_t readCount)
{
    if (fruDeviceId == 0xFF)
    {
        return ipmi::responseInvalidFieldRequest();
    }

    auto iter = frus.find(fruDeviceId);
    if (iter == frus.end())
    {
        return ipmi::responseSensorInvalid();
    }

    try
    {
        const auto& fruArea = getFruAreaData(fruDeviceId);
        auto size = fruArea.size();

        if (offset >= size)
        {
            return ipmi::responseParmOutOfRange();
        }

        // Write the count of response data.
        uint8_t returnCount;
        if ((offset + readCount) <= size)
        {
            returnCount = readCount;
        }
        else
        {
            returnCount = size - offset;
        }

        std::vector<uint8_t> fruData((fruArea.begin() + offset),
                                     (fruArea.begin() + offset + returnCount));

        return ipmi::responseSuccess(returnCount, fruData);
    }
    catch (const InternalFailure& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }
}

ipmi::RspType<uint8_t,  // SDR version
              uint16_t, // record count LS first
              uint16_t, // free space in bytes, LS first
              uint32_t, // addition timestamp LS first
              uint32_t, // deletion timestamp LS first
              uint8_t>  // operation Support
    ipmiGetRepositoryInfo()
{

    constexpr uint8_t sdrVersion = 0x51;
    constexpr uint16_t freeSpace = 0xFFFF;
    constexpr uint32_t additionTimestamp = 0x0;
    constexpr uint32_t deletionTimestamp = 0x0;
    constexpr uint8_t operationSupport = 0;

    uint16_t records = frus.size() + ipmi::sensor::sensors.size();

    return ipmi::responseSuccess(sdrVersion, records, freeSpace,
                                 additionTimestamp, deletionTimestamp,
                                 operationSupport);
}

void register_netfn_storage_functions()
{

    // <Get SEL Info>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetSelInfo, ipmi::Privilege::User,
                          ipmiStorageGetSelInfo);
    // <Get SEL Time>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetSelTime, ipmi::Privilege::User,
                          ipmiStorageGetSelTime);

    // <Set SEL Time>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdSetSelTime,
                          ipmi::Privilege::Operator, ipmiStorageSetSelTime);

    // <Reserve SEL>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdReserveSel, ipmi::Privilege::User,
                          ipmiStorageReserveSel);
    // <Get SEL Entry>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetSelEntry, ipmi::Privilege::User,
                          ipmiStorageGetSELEntry);

#ifndef JOURNAL_SEL
    // <Delete SEL Entry>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdDeleteSelEntry,
                          ipmi::Privilege::Operator, deleteSELEntry);
#endif
    // <Add SEL Entry>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdAddSelEntry,
                          ipmi::Privilege::Operator, ipmiStorageAddSEL);

    // <Clear SEL>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdClearSel, ipmi::Privilege::Operator,
                          ipmiStorageClearSEL);

    // <Get FRU Inventory Area Info>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetFruInventoryAreaInfo,
                          ipmi::Privilege::User, ipmiStorageGetFruInvAreaInfo);

    // <READ FRU Data>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdReadFruData,
                          ipmi::Privilege::Operator, ipmiStorageReadFruData);

    // <Get Repository Info>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetSdrRepositoryInfo,
                          ipmi::Privilege::User, ipmiGetRepositoryInfo);

    // <Reserve SDR Repository>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdReserveSdrRepository,
                          ipmi::Privilege::User, ipmiSensorReserveSdr);

    // <Get SDR>
    ipmi_register_callback(NETFUN_STORAGE, IPMI_CMD_GET_SDR, nullptr,
                           ipmi_sen_get_sdr, PRIVILEGE_USER);

    ipmi::fru::registerCallbackHandler();
    return;
}
