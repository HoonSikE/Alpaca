package com.example.taxi.ui.home.user

import android.annotation.SuppressLint
import android.util.Log
import android.view.View
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.databinding.FragmentUserHomeBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class UserHomeFragment: BaseFragment<FragmentUserHomeBinding>(R.layout.fragment_user_home) {
    private lateinit var destinationListAdapter: DestinationListAdapter
    private lateinit var favoritesAdapter: FavoritesAdapter
    private var favorites : MutableList<Favorites> = mutableListOf()
    private val userHomeViewModel : UserHomeViewModel by viewModels()
    private lateinit var destination: Destination
    private var addressFavorites = ""

    private val favoritesDeleteClickListener: (View, String) -> Unit = { _, address ->
        addressFavorites = address
        showFavoritesDialog(address)
    }

    private val destinationOnClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Destination(address,x,place,y)
        findNavController().navigate(R.id.action_userHomeFragment_to_destinationSettingFragment, bundleOf("Destination" to destination))
    }

    private val favoritesOnClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Destination(address,x,place,y)
        findNavController().navigate(R.id.action_userHomeFragment_to_destinationSettingFragment, bundleOf("Destination" to destination))
    }

    override fun init() {
        initAdapter()
        observerData()
        setOnClickListeners()
    }

    private fun initAdapter() {
        if(ApplicationClass.prefs.isEachProvider == true){
            binding.imageMoveProvider.show()
        }else{
            binding.imageMoveProvider.hide()
        }
        userHomeViewModel.getDestinations()
        destinationListAdapter = DestinationListAdapter().apply {
            onItemClickListener = destinationOnClickListener
        }
        binding.recyclerviewUserHomeDestinationList.layoutManager = LinearLayoutManager(requireContext())
        binding.recyclerviewUserHomeDestinationList.adapter = destinationListAdapter
        userHomeViewModel.getFavorites()
        favoritesAdapter = FavoritesAdapter().apply {
            onItemClickListener = favoritesDeleteClickListener
            onFavoritesClickListener = favoritesOnClickListener
        }
        binding.recyclerviewUserHomeFavorites.apply {
            adapter = favoritesAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
    }

    private fun observerData() {
        userHomeViewModel.destinations.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    binding.progressBar.show()
                }
                is UiState.Failure -> {
                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewUserHomeDestinationList.setBackgroundResource(R.drawable.layout_recycler_no_item)
                    binding.textUserHomeNoContentDestination.show()
                }
                is UiState.Success -> {
                    binding.progressBar.hide()
                    Glide.with(requireContext())
                        .load(ApplicationClass.prefs.profileImage)
                        .into(binding.imageUserHomeProfile)
                    binding.textUserHomeName.text = ApplicationClass.prefs.name + "님, 안녕하세요"
                    binding.textUserHomeCount.text = ApplicationClass.prefs.useCount.toString() + "회"
                    setLevel()
                    val list : MutableList<FrequentDestination> = state.data.toMutableList()
                    destinationListAdapter.updateList(list)
                    binding.recyclerviewUserHomeDestinationList.setBackgroundResource(R.drawable.layout_recycler)
                    binding.textUserHomeNoContentDestination.hide()
                }
            }
        }
        userHomeViewModel.favorites.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewUserHomeFavorites.setBackgroundResource(R.drawable.layout_recycler_no_item)
                    binding.textUserHomeNoContentFavorites.show()
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
                    favorites = state.data.toMutableList()
                    favoritesAdapter.updateList(favorites)
                    binding.recyclerviewUserHomeFavorites.setBackgroundResource(R.drawable.layout_recycler)
                    binding.textUserHomeNoContentFavorites.hide()
                }
            }
        }
    }

    private fun setOnClickListeners(){
        binding.imageMoveMyPage.setOnClickListener{
            findNavController().navigate(R.id.action_userHomeFragment_to_myPageFragment)
        }
        binding.imageMoveProvider.setOnClickListener {
            findNavController().navigate(R.id.action_userHomeFragment_to_providerHomeFragment)
        }
        binding.buttonUserHomeCallTaxi.setOnClickListener {
            findNavController().navigate(R.id.action_userHomeFragment_to_startPointSettingFragment)
        }
    }

    private fun showFavoritesDialog(address: String) {
        FavoritesDialogFragment { favoritesListener(address) }.show(childFragmentManager, "FAVORITES_DIALOG")
    }

    private val favoritesListener: (address: String) -> Unit = {
        if(favorites.size < 2) {
            userHomeViewModel.deleteFavorites()
        }else{
            for(i in 0 until favorites.size){
                if(favorites[i].address == addressFavorites) {
                    favorites.removeAt(i)
                    userHomeViewModel.updateFavorites(favorites)
                }
            }

        }
        userHomeViewModel.getFavorites()
    }

    @SuppressLint("ResourceAsColor")
    private fun setLevel(){
        when(ApplicationClass.prefs.useCount?.div(10)){
            1 -> {
                binding.textUserHomeClass.text = "Bronze"
                binding.textUserHomeClass.setTextColor(R.color.bronze)
            }
            2 -> {
                binding.textUserHomeClass.text = "Silver"
                binding.textUserHomeClass.setTextColor(R.color.silver)
            }
            3 -> {
                binding.textUserHomeClass.text = "Gold"
                binding.textUserHomeClass.setTextColor(R.color.gold)
            }
            4 -> {
                binding.textUserHomeClass.text = "Platinum"
                binding.textUserHomeClass.setTextColor(R.color.platinum)
            }
            5 -> {
                binding.textUserHomeClass.text = "Diamond"
                binding.textUserHomeClass.setTextColor(R.color.diamond)
            }
        }
    }
}