#include "twilio.hpp"
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>

bool Twilio::send_message(
    const String& to_number,
    const String& from_number,
    const String& message_body,
    String& response,
    const String& media_url
) {
    const char* host = "api.twilio.com";
    String url = String("https://") + host + "/2010-04-01/Accounts/" + account_sid + "/Messages.json";

    // Prepare secure client with fingerprint
    BearSSL::WiFiClientSecure client;
    client.setFingerprint(fingerprint.c_str());

    // Initialize HTTPS
    HTTPClient https;
    if (!https.begin(client, url)) {
        response = "Unable to begin HTTPS connection";
        return false;
    }

    // HTTP Basic Auth and headers
    https.setAuthorization(account_sid.c_str(), auth_token.c_str());
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // Build payload with URL-encoding
    String body = "To=" + urlencode(to_number)
                + "&From=" + urlencode(from_number)
                + "&Body=" + urlencode(message_body);
    if (media_url.length()) {
        body += "&MediaUrl=" + urlencode(media_url);
    }

    // Send POST request
    int httpCode = https.POST(body);
    response = https.getString();
    https.end();

    // Success codes: 200 OK or 201 Created
    return (httpCode == 200 || httpCode == 201);
}

/* Private function to create a Basic Auth field and parameter */
String Twilio::_get_auth_header(const String& user, const String& password) {
        size_t toencodeLen = user.length() + password.length() + 2;
        char toencode[toencodeLen];
        memset(toencode, 0, toencodeLen);
        snprintf(
                toencode,
                toencodeLen,
                "%s:%s",
                user.c_str(),
                password.c_str()
        );

        String encoded = base64::encode((uint8_t*)toencode, toencodeLen-1);
        String encoded_string = String(encoded);
        size_t i = 0;

        // Strip newlines (after every 72 characters in spec)
        while (i < encoded_string.length()) {
                i = encoded_string.indexOf('\n', i);
                if (i == -1) {
                        break;
                }
                encoded_string.remove(i, 1);
        }
        return "Authorization: Basic " + encoded_string;
}
