#include "update_checker.h"
#include "pch.h"
#include "utils/logger.h"
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#include <winhttp.h>
#include <shellapi.h>

#pragma comment(lib, "winhttp.lib")

namespace UpdateChecker {
    const std::string CURRENT_VERSION = "0.9.3";

    static std::string TrimPrefixV(std::string v) {
        if (!v.empty() && (v[0] == 'v' || v[0] == 'V')) {
            v.erase(v.begin());
        }
        return v;
    }

    static std::vector<uint32_t> ParseVersionParts(const std::string& version) {
        std::vector<uint32_t> parts;
        std::string v = TrimPrefixV(version);

        uint64_t acc = 0;
        bool inNumber = false;
        for (char c : v) {
            if (c >= '0' && c <= '9') {
                inNumber = true;
                acc = acc * 10 + (uint64_t)(c - '0');
            }
            else {
                if (inNumber) {
                    parts.push_back((uint32_t)std::min<uint64_t>(acc, UINT32_MAX));
                    acc = 0;
                    inNumber = false;
                }
                if (c == '.') {
                    continue;
                }
                break;
            }
        }
        if (inNumber) {
            parts.push_back((uint32_t)std::min<uint64_t>(acc, UINT32_MAX));
        }

        while (parts.size() < 3) {
            parts.push_back(0);
        }
        return parts;
    }

    static int CompareVersions(const std::string& a, const std::string& b) {
        const auto pa = ParseVersionParts(a);
        const auto pb = ParseVersionParts(b);
        const size_t n = std::max(pa.size(), pb.size());
        for (size_t i = 0; i < n; ++i) {
            const uint32_t ai = i < pa.size() ? pa[i] : 0;
            const uint32_t bi = i < pb.size() ? pb[i] : 0;
            if (ai < bi) return -1;
            if (ai > bi) return 1;
        }
        return 0;
    }

    static std::string LoadIgnoredVersionOnce() {
        char path[MAX_PATH] = {};
        if (GetModuleFileNameA(nullptr, path, MAX_PATH) == 0) {
            return {};
        }
        std::string exePath(path);
        const size_t lastSlash = exePath.find_last_of("\\/");
        const std::string dir = (lastSlash == std::string::npos) ? std::string() : exePath.substr(0, lastSlash + 1);
        const std::string filePath = dir + "BetterVR_update_ignore_once.txt";

        std::ifstream f(filePath, std::ios::in);
        if (!f.is_open()) {
            return {};
        }
        std::string ignored;
        std::getline(f, ignored);
        return ignored;
    }

    static void SaveIgnoredVersionOnce(const std::string& version) {
        char path[MAX_PATH] = {};
        if (GetModuleFileNameA(nullptr, path, MAX_PATH) == 0) {
            return;
        }
        std::string exePath(path);
        const size_t lastSlash = exePath.find_last_of("\\/");
        const std::string dir = (lastSlash == std::string::npos) ? std::string() : exePath.substr(0, lastSlash + 1);
        const std::string filePath = dir + "BetterVR_update_ignore_once.txt";

        std::ofstream f(filePath, std::ios::out | std::ios::trunc);
        if (!f.is_open()) {
            return;
        }
        f << version;
    }

    void CheckForUpdatesAsync() {
        HINTERNET hSession = NULL, hConnect = NULL, hRequest = NULL;

        hSession = WinHttpOpen(L"BotW-BetterVR Update Checker", WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
        if (!hSession) {
            Log::print<ERROR>("UpdateChecker: Failed to open WinHTTP session.");
            return;
        }

        hConnect = WinHttpConnect(hSession, L"api.github.com", INTERNET_DEFAULT_HTTPS_PORT, 0);
        if (!hConnect) {
            Log::print<ERROR>("UpdateChecker: Failed to connect to GitHub API.");
            WinHttpCloseHandle(hSession);
            return;
        }

        hRequest = WinHttpOpenRequest(hConnect, L"GET", L"/repos/Crementif/BotW-BetterVR/releases/latest", NULL, WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE);
        if (!hRequest) {
            Log::print<ERROR>("UpdateChecker: Failed to open request.");
            WinHttpCloseHandle(hConnect);
            WinHttpCloseHandle(hSession);
            return;
        }

        BOOL bResults = WinHttpSendRequest(hRequest, WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0);
        if (bResults) {
            bResults = WinHttpReceiveResponse(hRequest, NULL);
        }
        else {
            Log::print<ERROR>("UpdateChecker: Failed to send request.");
        }

        if (bResults) {
            DWORD dwSize = 0;
            DWORD dwDownloaded = 0;
            std::string response;

            do {
                dwSize = 0;
                if (!WinHttpQueryDataAvailable(hRequest, &dwSize)) {
                    Log::print<ERROR>("UpdateChecker: Error in WinHttpQueryDataAvailable.");
                    break;
                }

                if (dwSize == 0)
                    break;

                std::vector<char> buffer(dwSize + 1);
                if (!WinHttpReadData(hRequest, (LPVOID)buffer.data(), dwSize, &dwDownloaded)) {
                    Log::print<ERROR>("UpdateChecker: Error in WinHttpReadData.");
                    break;
                }

                response.append(buffer.data(), dwDownloaded);

            } while (dwSize > 0);

            // extracts the JSON string value for the "tag_name" key
            const std::string searchKey = "\"tag_name\"";
            const size_t pos = response.find(searchKey);
            if (pos != std::string::npos) {
                const size_t colon = response.find(':', pos);
                if (colon != std::string::npos) {
                    const size_t valueStart = response.find('"', colon);
                    if (valueStart != std::string::npos) {
                        const size_t valueEnd = response.find('"', valueStart + 1);
                        if (valueEnd != std::string::npos) {
                            const std::string latestVersion = response.substr(valueStart + 1, valueEnd - valueStart - 1);

                            if (CompareVersions(CURRENT_VERSION, latestVersion) < 0) {
                                const std::string ignoredOnce = LoadIgnoredVersionOnce();
                                if (ignoredOnce == latestVersion) {
                                    Log::print<INFO>("UpdateChecker: Update {} ignored once.", latestVersion);
                                }
                                else {
                                    Log::print<INFO>("UpdateChecker: A new version is available! Current: {}, Latest: {}", CURRENT_VERSION, latestVersion);
                                    Log::print<INFO>("UpdateChecker: Please download the latest version from https://github.com/Crementif/BotW-BetterVR/releases");

                                    std::string mbText;
                                    mbText += "A new version of BotW-BetterVR is available.\n\n";
                                    mbText += "Current: " + CURRENT_VERSION + "\n";
                                    mbText += "Latest:  " + latestVersion + "\n\n";
                                    mbText += "Open releases page?\n\n";
                                    mbText += "Yes = Open page\nNo = Dismiss\nCancel = Ignore this update once";

                                    const int res = MessageBoxA(NULL, mbText.c_str(), "BetterVR Update Available", MB_YESNOCANCEL | MB_ICONINFORMATION);
                                    if (res == IDYES) {
                                        ShellExecuteA(NULL, "open", "https://github.com/Crementif/BotW-BetterVR/releases", NULL, NULL, SW_SHOWNORMAL);
                                    }
                                    else if (res == IDCANCEL) {
                                        SaveIgnoredVersionOnce(latestVersion);
                                    }
                                }
                            }
                            else {
                                Log::print<INFO>("UpdateChecker: No update available. Current: {}, Latest: {}", CURRENT_VERSION, latestVersion);
                            }
                        }
                    }
                }
            }
            else {
                Log::print<WARNING>("UpdateChecker: Could not find tag_name in response.");
            }
        }
        else {
            Log::print<ERROR>("UpdateChecker: Failed to receive response. Error %d", GetLastError());
        }

        if (hRequest) WinHttpCloseHandle(hRequest);
        if (hConnect) WinHttpCloseHandle(hConnect);
        if (hSession) WinHttpCloseHandle(hSession);
    }

    void CheckForUpdates() {
#ifndef _DEBUG
        std::thread(CheckForUpdatesAsync).detach();
#endif
    }
}
