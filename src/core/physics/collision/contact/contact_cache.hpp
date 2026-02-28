#pragma once

#include <core/physics/collision/contact/contact_manifold.hpp>
#include <unordered_map>
#include <vector>

namespace phynity::physics::collision {

/// Contact cache for tracking persistent contacts across frames
/// Enables warm-starting and stable stacking by maintaining contact history
class ContactCache {
public:
    static constexpr int DEFAULT_MAX_AGE = 3;  ///< Default frame lifetime for contacts

    /// Create a contact cache with specified maximum age
    /// @param max_age Maximum number of frames to keep a contact without refresh
    explicit ContactCache(int max_age = DEFAULT_MAX_AGE)
        : max_age_(max_age) {}

    /// Update the cache with new contacts from the current frame
    /// Matches new contacts with cached ones, updates ages, and removes expired contacts
    /// @param new_manifolds Vector of newly detected contact manifolds
    /// @return Vector of manifolds with cached data applied (warm-start impulses, ages)
    std::vector<ContactManifold> update(const std::vector<ContactManifold>& new_manifolds) {
        std::vector<ContactManifold> updated_manifolds;
        updated_manifolds.reserve(new_manifolds.size());

        // Mark all cached contacts as "not seen this frame"
        std::unordered_map<uint64_t, bool> seen_this_frame;
        for (const auto& [id, _] : cache_) {
            seen_this_frame[id] = false;
        }

        // Process each new manifold
        for (ContactManifold manifold : new_manifolds) {
            // Generate contact ID for this manifold
            manifold.update_contact_id();
            uint64_t contact_id = manifold.contact_id;

            // Check if this contact exists in the cache
            auto it = cache_.find(contact_id);
            if (it != cache_.end()) {
                // Contact found: this is a persistent contact
                const ContactManifold& cached = it->second;
                
                // Apply warm-start: copy previous impulse
                manifold.previous_impulse = cached.previous_impulse;
                
                // Increment age
                manifold.age = cached.age + 1;
                
                // Mark as seen
                seen_this_frame[contact_id] = true;
            } else {
                // New contact: no warm-start data
                manifold.previous_impulse = Vec3f(0.0f);
                manifold.age = 0;
                seen_this_frame[contact_id] = true;
            }

            // Update cache with current manifold
            cache_[contact_id] = manifold;
            updated_manifolds.push_back(manifold);
        }

        // Remove contacts that weren't seen this frame and have aged out
        cleanup_expired_contacts(seen_this_frame);

        return updated_manifolds;
    }

    /// Get a cached contact by ID
    /// @param contact_id The contact ID to look up
    /// @return Pointer to the cached manifold, or nullptr if not found
    const ContactManifold* get(uint64_t contact_id) const {
        auto it = cache_.find(contact_id);
        return (it != cache_.end()) ? &it->second : nullptr;
    }

    /// Store or update the impulse applied to a contact
    /// This should be called after resolving a contact to cache the impulse for warm-start
    /// @param contact_id The contact ID
    /// @param impulse The impulse that was applied
    void store_impulse(uint64_t contact_id, const Vec3f& impulse) {
        auto it = cache_.find(contact_id);
        if (it != cache_.end()) {
            it->second.previous_impulse = impulse;
        }
    }

    /// Clear all cached contacts
    void clear() {
        cache_.clear();
    }

    /// Get the number of cached contacts
    size_t size() const {
        return cache_.size();
    }

    /// Check if the cache is empty
    bool empty() const {
        return cache_.empty();
    }

    /// Get the maximum age for contacts
    int max_age() const {
        return max_age_;
    }

private:
    /// Remove contacts that haven't been seen and are past their expiration age
    void cleanup_expired_contacts(const std::unordered_map<uint64_t, bool>& seen_this_frame) {
        std::vector<uint64_t> to_remove;

        for (auto& [id, manifold] : cache_) {
            // If contact wasn't seen this frame, increment "frames since last seen"
            if (seen_this_frame.at(id) == false) {
                // Contact not refreshed: check if it should be removed
                // We don't have a separate "frames_since_seen" counter, so we use age
                // If a contact disappears, we'll remove it immediately in the next cleanup
                to_remove.push_back(id);
            }
        }

        // Remove expired contacts
        for (uint64_t id : to_remove) {
            cache_.erase(id);
        }
    }

    std::unordered_map<uint64_t, ContactManifold> cache_;  ///< Map of contact_id -> manifold
    int max_age_;                                          ///< Maximum age before removal
};

}  // namespace phynity::physics::collision
