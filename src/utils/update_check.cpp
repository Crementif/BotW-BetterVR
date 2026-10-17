#include "pch.h"
#include "update_check.h"
#include "bettervr_version.h"

#include <curl/curl.h>

static constexpr const char* kLatestReleaseUrl = "https://api.github.com/repos/Crementif/BotW-BetterVR/releases/latest";

static std::mutex s_mutex;
static std::optional<std::string> s_newerRelease;

static std::array<uint32_t, 3> ParseVersion(std::string_view version) {
    if (!version.empty() && (version.front() == 'v' || version.front() == 'V')) {
        version.remove_prefix(1);
    }
    std::array<uint32_t, 3> parts = {};
    for (uint32_t& part : parts) {
        const auto [end, error] = std::from_chars(version.data(), version.data() + version.size(), part);
        if (error != std::errc() || end == version.data() + version.size() || *end != '.') {
            break;
        }
        version.remove_prefix(end - version.data() + 1);
    }
    return parts;
}

static size_t AppendResponse(char* data, size_t size, size_t count, void* response) {
    static_cast<std::string*>(response)->append(data, size * count);
    return size * count;
}

static std::optional<std::string> FetchLatestTag() {
    CURL* curl = curl_easy_init();
    if (curl == nullptr) {
        return std::nullopt;
    }
    std::string response;
    curl_slist* headers = curl_slist_append(nullptr, "Accept: application/vnd.github+json");
    curl_easy_setopt(curl, CURLOPT_URL, kLatestReleaseUrl);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "BotW-BetterVR");
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, AppendResponse);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    const CURLcode result = curl_easy_perform(curl);
    long status = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    if (result != CURLE_OK || status != 200) {
        Log::print<WARNING>("Couldn't check for a newer BetterVR release: {} (HTTP {})", curl_easy_strerror(result), status);
        return std::nullopt;
    }

    const size_t key = response.find("\"tag_name\"");
    const size_t open = key == std::string::npos ? key : response.find('"', response.find(':', key));
    const size_t close = open == std::string::npos ? open : response.find('"', open + 1);
    if (close == std::string::npos) {
        return std::nullopt;
    }
    return response.substr(open + 1, close - open - 1);
}

void UpdateCheck::Start() {
#if !defined(_DEBUG)
    std::thread([] {
        const std::optional<std::string> latest = FetchLatestTag();
        if (!latest || ParseVersion(*latest) <= ParseVersion(BETTERVR_VERSION)) {
            return;
        }
        Log::print<INFO>("BetterVR {} is out, this is {}", *latest, BETTERVR_VERSION_STRING);
        std::scoped_lock lock(s_mutex);
        s_newerRelease = latest;
    }).detach();
#endif
}

std::optional<std::string> UpdateCheck::GetNewerRelease() {
    std::scoped_lock lock(s_mutex);
    return s_newerRelease;
}
