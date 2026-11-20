// Edits of move_group's AllowedCollisionMatrix message that keep the meaning MoveIt gives it.
//
// /get_planning_scene lists each pair with its effective value: the pair's own entry if it has
// one, otherwise what the default entries give. /apply_planning_scene turns the message back
// into a matrix, and keeps a listed value as the pair's own entry only where it differs from
// the default entries (MoveIt's AllowedCollisionMatrix message constructor). So an edit that
// only changes a default entry leaves the listed pairs at their old effective values, and they
// come back as entries of their own that outlast the change. Each edit here rewrites the listed
// pairs it decides, no edit relies on a default entry alone, and each keeps every name with a
// default entry listed (keepNamesListed).

#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>

namespace collision_matrix_edit
{
using Matrix = moveit_msgs::msg::AllowedCollisionMatrix;

// Whether a pair may touch apart from any default entry, for example because the SRDF
// disables it or the commander allows it.
using Baseline = std::function<bool(const std::string &, const std::string &)>;

inline std::optional<std::size_t> indexOf(const std::vector<std::string> & names,
                                          const std::string & name)
{
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (names[i] == name) return i;
    }
    return std::nullopt;
}

// The default entry of `name`, if it has one.
inline std::optional<bool> defaultEntry(const Matrix & acm, const std::string & name)
{
    const auto i = indexOf(acm.default_entry_names, name);
    if (!i || *i >= acm.default_entry_values.size()) return std::nullopt;
    return static_cast<bool>(acm.default_entry_values[*i]);
}

// Whether the default entry of `name` lets it touch anything, as the toggles below set it.
inline bool defaultAllows(const Matrix & acm, const std::string & name)
{
    return defaultEntry(acm, name).value_or(false);
}

// Whether move_group lets `a` and `b` touch, read the way MoveIt reads the message: the listed
// value when both names are listed, otherwise the default entries, where one that forbids wins
// (AllowedCollisionMatrix::getDefaultEntry).
inline bool allows(const Matrix & acm, const std::string & a, const std::string & b)
{
    const auto ia = indexOf(acm.entry_names, a);
    const auto ib = indexOf(acm.entry_names, b);
    if (ia && ib && *ia < acm.entry_values.size() && *ib < acm.entry_values[*ia].enabled.size()) {
        return acm.entry_values[*ia].enabled[*ib];
    }
    const auto da = defaultEntry(acm, a);
    const auto db = defaultEntry(acm, b);
    if (da && db) return *da && *db;
    if (da) return *da;
    if (db) return *db;
    return false;
}

// Lists `name` and returns its index. A name the message leaves out follows the default
// entries, so its new row takes the values they give, and listing it changes nothing.
inline std::size_t list(Matrix & acm, const std::string & name)
{
    if (const auto found = indexOf(acm.entry_names, name)) return *found;
    std::vector<bool> row;
    row.reserve(acm.entry_names.size() + 1);
    for (const auto & other : acm.entry_names) row.push_back(allows(acm, name, other));
    row.push_back(false);
    for (std::size_t i = 0; i < acm.entry_values.size(); ++i) {
        acm.entry_values[i].enabled.push_back(row[i]);
    }
    acm.entry_names.push_back(name);
    moveit_msgs::msg::AllowedCollisionEntry entry;
    entry.enabled = row;
    acm.entry_values.push_back(entry);
    return acm.entry_names.size() - 1;
}

inline void setDefault(Matrix & acm, const std::string & name, bool allowed)
{
    if (const auto i = indexOf(acm.default_entry_names, name)) {
        acm.default_entry_values[*i] = allowed;
        return;
    }
    acm.default_entry_names.push_back(name);
    acm.default_entry_values.push_back(allowed);
}

// What a pair should be: allowed by the baseline, or by the default entry of either name.
inline bool intended(const Matrix & acm, const Baseline & baseline,
                     const std::string & a, const std::string & b)
{
    return baseline(a, b) || defaultAllows(acm, a) || defaultAllows(acm, b);
}

inline void setPair(Matrix & acm, const std::string & a, const std::string & b, bool allowed)
{
    const std::size_t ia = list(acm, a);
    const std::size_t ib = list(acm, b);
    acm.entry_values[ia].enabled[ib] = allowed;
    acm.entry_values[ib].enabled[ia] = allowed;
}

// Two names no body has, which sort after any real one: '~' is the last printable ASCII
// character.
inline constexpr const char * kListingAnchors[2] = {"~listing_anchor_a", "~listing_anchor_b"};

// Keeps every name with a default entry in the message /get_planning_scene sends back.
// MoveIt's getMessage leaves out a name that has a default entry but no entry of its own
// when it sorts after every name that has one (AllowedCollisionMatrix::getAllEntryNames),
// and the next edit then drops that default entry. Once every name is ignored, every pair
// equals the default entries, no entry is left, and the matrix reads back empty. The
// anchors' pair is allowed while no default entry allows it, so it stays an entry of its
// own that sorts last.
inline void keepNamesListed(Matrix & acm)
{
    setPair(acm, kListingAnchors[0], kListingAnchors[1], true);
}

// Rewrites each pair of `a` with one of `others` to what it should be.
inline void refresh(Matrix & acm, const std::string & a, const std::vector<std::string> & others,
                    const Baseline & baseline)
{
    for (const auto & b : others) {
        if (b != a) setPair(acm, a, b, intended(acm, baseline, a, b));
    }
    keepNamesListed(acm);
}

// Turns collision checking of `names` against everything off (ignore = true) or back on.
// Each name's default entry records the choice and covers names the message does not list,
// such as collision objects added later. Every listed pair of each name is then rewritten.
// Every name with a default entry is listed first: the message can leave one out (MoveIt's
// getAllEntryNames skips a default-only name that sorts last), and then the pairs between
// it and `names` would follow the default entries, where one that forbids wins.
inline void setIgnored(Matrix & acm, const std::vector<std::string> & names, bool ignore,
                       const Baseline & baseline)
{
    for (const auto & name : names) setDefault(acm, name, ignore);
    const std::vector<std::string> with_defaults = acm.default_entry_names;
    for (const auto & name : with_defaults) list(acm, name);
    for (const auto & name : names) {
        const std::vector<std::string> listed = acm.entry_names;
        refresh(acm, name, listed, baseline);
    }
}
}  // namespace collision_matrix_edit
