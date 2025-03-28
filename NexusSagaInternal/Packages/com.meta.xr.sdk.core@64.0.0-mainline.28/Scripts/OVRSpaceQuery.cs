/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using System;
using System.Collections.Generic;

/// <summary>
/// Utility to assist with queries for <see cref="OVRSpace"/>s.
/// </summary>
internal static class OVRSpaceQuery
{
    /// <summary>
    /// Represents options used to generate an <see cref="OVRSpaceQuery"/>.
    /// </summary>
    public struct Options
    {
        /// <summary>
        /// The maximum number of UUIDs which can be used in a <see cref="UuidFilter"/>.
        /// </summary>
        public const int MaxUuidCount = OVRPlugin.SpaceFilterInfoIdsMaxSize;

        private static readonly Guid[] Ids = new Guid[MaxUuidCount];

        private static readonly OVRPlugin.SpaceComponentType[] ComponentTypes =
            new OVRPlugin.SpaceComponentType[OVRPlugin.SpaceFilterInfoComponentsMaxSize];

        /// <summary>
        /// The maximum number of results the query can return.
        /// </summary>
        public int MaxResults { get; set; }

        /// <summary>
        /// The timeout, in seconds for the query.
        /// </summary>
        /// <remarks>
        /// Zero indicates the query does not timeout.
        /// </remarks>
        public double Timeout { get; set; }

        /// <summary>
        /// The storage location to query.
        /// </summary>
        public OVRSpace.StorageLocation Location { get; set; }

        /// <summary>
        /// The type of query to perform.
        /// </summary>
        public OVRPlugin.SpaceQueryType QueryType { get; set; }

        /// <summary>
        /// The type of action to perform.
        /// </summary>
        public OVRPlugin.SpaceQueryActionType ActionType { get; set; }

        private OVRPlugin.SpaceComponentType _componentType;

        private IEnumerable<Guid> _uuidFilter;

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE

        private Guid? _localGroupUuidFilter;

#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

        /// <summary>
        /// The components which must be present on the space in order to match the query.
        /// </summary>
        /// <remarks>
        /// The query will be limited to spaces that have this set of components. You may filter by component type or
        /// UUID (see <see cref="UuidFilter"/>), but not both at the same time.
        ///
        /// Currently, only one component is allowed at a time.
        /// </remarks>
        /// <exception cref="InvalidOperationException">Thrown if <see cref="UuidFilter"/> is not `null`.</exception>
        public OVRPlugin.SpaceComponentType ComponentFilter
        {
            get => _componentType;
            set
            {
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
                ValidateSingleFilter(_uuidFilter, value, _localGroupUuidFilter);
#else
                ValidateSingleFilter(_uuidFilter, value);
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

                _componentType = value;
            }
        }

        /// <summary>
        /// A set of UUIDs used to filter the query.
        /// </summary>
        /// <remarks>
        /// The query will look for this set of UUIDs and only return matching UUIDs up to <see cref="MaxResults"/>.
        /// You may filter by component type (see <see cref="ComponentFilter"/>) or UUIDs, but not both at the same
        /// time.
        /// </remarks>
        /// <exception cref="InvalidOperationException">Thrown if <see cref="ComponentFilter"/> is not 0.</exception>
        /// <exception cref="ArgumentException">Thrown if <see cref="UuidFilter"/> is set to a value that contains more
        /// than <seealso cref="MaxUuidCount"/> UUIDs.</exception>
        public IEnumerable<Guid> UuidFilter
        {
            get => _uuidFilter;
            set
            {
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
                ValidateSingleFilter(value, _componentType, _localGroupUuidFilter);
#else
                ValidateSingleFilter(value, _componentType);
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

                if (value is IReadOnlyCollection<Guid> collection && collection.Count > MaxUuidCount)
                    throw new ArgumentException(
                        $"There must not be more than {MaxUuidCount} UUIDs specified by the {nameof(UuidFilter)} (new value contains {collection.Count} UUIDs).",
                        nameof(value));

                _uuidFilter = value;
            }
        }

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE

        /// <summary>
        /// The Local Group UUID to query. Will be null if not querying a Local Group.
        /// </summary>
        public Guid? LocalGroupUuidFilter
        {
            get => _localGroupUuidFilter;
            set
            {
                ValidateSingleFilter(_uuidFilter, _componentType, value);
                _localGroupUuidFilter = value;
            }
        }

#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

        /// <summary>
        /// Creates a copy of <paramref name="other"/>.
        /// </summary>
        /// <param name="other">The options to copy.</param>
        public Options(Options other)
        {
            QueryType = other.QueryType;
            MaxResults = other.MaxResults;
            Timeout = other.Timeout;
            Location = other.Location;
            ActionType = other.ActionType;
            _componentType = other._componentType;
            _uuidFilter = other._uuidFilter;
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
            _localGroupUuidFilter = other._localGroupUuidFilter;
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE
        }

        /// <summary>
        /// Creates a new <see cref="OVRPlugin.SpaceQueryInfo"/> from this.
        /// </summary>
        /// <returns>The newly created info.</returns>
        public OVRPlugin.SpaceQueryInfo ToQueryInfo()
        {
            var filterType = OVRPlugin.SpaceQueryFilterType.None;
            var numIds = 0;
            var numComponents = 0;
            if (_uuidFilter != null)
            {
                filterType = OVRPlugin.SpaceQueryFilterType.Ids;
                foreach (var id in _uuidFilter.ToNonAlloc())
                {
                    if (numIds >= MaxUuidCount)
                        throw new InvalidOperationException(
                            $"{nameof(UuidFilter)} must not contain more than {MaxUuidCount} UUIDs.");

                    Ids[numIds++] = id;
                }
            }
            else
            {
                filterType = OVRPlugin.SpaceQueryFilterType.Components;
                ComponentTypes[numComponents++] = _componentType;
            }

            return new OVRPlugin.SpaceQueryInfo
            {
                QueryType = QueryType,
                MaxQuerySpaces = MaxResults,
                Timeout = Timeout,
                Location = Location.ToSpaceStorageLocation(),
                ActionType = ActionType,
                FilterType = filterType,
                IdInfo = new OVRPlugin.SpaceFilterInfoIds
                {
                    Ids = Ids,
                    NumIds = numIds
                },
                ComponentsInfo = new OVRPlugin.SpaceFilterInfoComponents
                {
                    Components = ComponentTypes,
                    NumComponents = numComponents,
                }
            };
        }

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE

        /// <summary>
        /// Creates a new <see cref="OVRPlugin.SpaceQueryInfo2"/> from this.
        /// </summary>
        /// <returns>The newly created info.</returns>
        public OVRPlugin.SpaceQueryInfo2 ToQueryInfo2()
        {
            var filterType = OVRPlugin.SpaceQueryFilterType.None;
            var numIds = 0;
            var numComponents = 0;
            if (_uuidFilter != null)
            {
                filterType = OVRPlugin.SpaceQueryFilterType.Ids;
                foreach (var id in _uuidFilter.ToNonAlloc())
                {
                    if (numIds >= MaxUuidCount)
                        throw new InvalidOperationException(
                            $"{nameof(UuidFilter)} must not contain more than {MaxUuidCount} UUIDs.");

                    Ids[numIds++] = id;
                }
            }
            else if (_localGroupUuidFilter != null)
            {
                filterType = OVRPlugin.SpaceQueryFilterType.LocalGroup;
            }
            else
            {
                filterType = OVRPlugin.SpaceQueryFilterType.Components;
                ComponentTypes[numComponents++] = _componentType;
            }

            return new OVRPlugin.SpaceQueryInfo2
            {
                QueryType = QueryType,
                MaxQuerySpaces = MaxResults,
                Timeout = Timeout,
                Location = Location.ToSpaceStorageLocation(),
                LocalGroupInfo = new OVRPlugin.SpaceFilterInfoLocalGroup
                {
                    GroupUuid = _localGroupUuidFilter ?? Guid.Empty,
                },
                ActionType = ActionType,
                FilterType = filterType,
                IdInfo = new OVRPlugin.SpaceFilterInfoIds
                {
                    Ids = Ids,
                    NumIds = numIds
                },
                ComponentsInfo = new OVRPlugin.SpaceFilterInfoComponents
                {
                    Components = ComponentTypes,
                    NumComponents = numComponents,
                }
            };
        }

#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

        /// <summary>
        /// Initiates a space query.
        /// </summary>
        /// <param name="requestId">When this method returns, <paramref name="requestId"/> will represent a valid
        /// request if successful, or an invalid request if not. This parameter is passed initialized.</param>
        /// <returns>`true` if the query was successfully started; otherwise, `false`.</returns>
        public bool TryQuerySpaces(out ulong requestId)
        {
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
            var querySpaces = OVRPlugin.QuerySpaces2(ToQueryInfo2(), out requestId);
#else
            var querySpaces = OVRPlugin.QuerySpaces(ToQueryInfo(), out requestId);
#endif // OVR_PARTNER_CODE || OVR_INTERNAL_CODE

            return querySpaces;
        }

#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE
        private static void ValidateSingleFilter(IEnumerable<Guid> uuidFilter, OVRPlugin.SpaceComponentType componentFilter, Guid? localGroupUuidFilter)
        {
            var filterCount = 0;
            if (uuidFilter != null)
                ++filterCount;
            if (componentFilter != 0)
                ++filterCount;
            if (localGroupUuidFilter != null)
                ++filterCount;
            if (filterCount > 1)
                throw new InvalidOperationException($"You may only query by UUID, component type, or Local Group.");
        }
#else
        private static void ValidateSingleFilter(IEnumerable<Guid> uuidFilter, OVRPlugin.SpaceComponentType componentFilter)
        {
            if (uuidFilter != null && componentFilter != 0)
                throw new InvalidOperationException($"You may only query by UUID or by component type.");
        }
#endif
    }
}
