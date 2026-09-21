#include "JpegCommentParser.h"

#include <cstdlib>
#include <cstring>

namespace {

// Parse "KEY=VALUE" lines from one COM payload into the snapshot.
void parseCommentText(const std::string & text, SensorSnapshot & snap){
	size_t pos = 0;
	while(pos < text.size()){
		size_t eol = text.find_first_of("\r\n", pos);
		if(eol == std::string::npos) eol = text.size();
		if(eol > pos){
			const std::string line = text.substr(pos, eol - pos);
			const size_t eq = line.find('=');
			if(eq != std::string::npos && eq > 0){
				const std::string key = line.substr(0, eq);
				const std::string value = line.substr(eq + 1);
				const int intValue = std::atoi(value.c_str());
				if(key == "LA2") snap.la2 = intValue;
				else if(key == "LXL") snap.lxl = intValue;
				else if(key == "LXR") snap.lxr = intValue;
				else if(key == "PI2") snap.pi2 = intValue;
				else if(key == "TC2") snap.tc2 = intValue;
				else if(key == "TIN") snap.tin = intValue;
				else if(key == "FRJ") snap.frj = intValue;
				else if(key == "XTO") snap.width = intValue;
				else if(key == "YTO") snap.height = intValue;
				else if(key == "CAM") snap.cam = value;
			}
		}
		pos = eol;
		while(pos < text.size() && (text[pos] == '\r' || text[pos] == '\n')) pos++;
	}
}

// Heuristic: the Mobotix "MXF" comment segment is binary — skip it in dumps.
bool looksLikeText(const uint8_t * p, size_t n){
	if(n == 0) return false;
	size_t printable = 0;
	const size_t sample = n < 64 ? n : 64;
	for(size_t i = 0; i < sample; i++){
		const uint8_t c = p[i];
		if(c == '\r' || c == '\n' || c == '\t' || (c >= 0x20 && c < 0x7F)) printable++;
	}
	return printable * 10 >= sample * 9; // >= 90% printable
}

} // namespace

SensorSnapshot JpegCommentParser::parse(const uint8_t * data, size_t size, std::string * comDump){
	SensorSnapshot snap;
	if(comDump) comDump->clear();
	if(size < 4 || data[0] != 0xFF || data[1] != 0xD8) return snap;

	size_t i = 2;
	while(i + 4 <= size){
		if(data[i] != 0xFF) break;
		const uint8_t marker = data[i + 1];
		if(marker == 0xD8 || marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)){
			i += 2;
			continue;
		}
		if(marker == 0xD9 || marker == 0xDA) break; // EOI or start of scan — no more headers
		const size_t segLen = (static_cast<size_t>(data[i + 2]) << 8) | data[i + 3];
		if(segLen < 2 || i + 2 + segLen > size) break;
		if(marker == 0xFE){ // COM
			const uint8_t * payload = data + i + 4;
			const size_t payloadLen = segLen - 2;
			if(looksLikeText(payload, payloadLen)){
				const std::string text(reinterpret_cast<const char *>(payload), payloadLen);
				parseCommentText(text, snap);
				if(comDump){
					comDump->append(text);
					comDump->append("\n");
				}
			}
		}
		i += 2 + segLen;
	}

	snap.valid = (snap.width > 0 || snap.la2 >= 0 || snap.lxl >= 0 || snap.lxr >= 0);
	return snap;
}
